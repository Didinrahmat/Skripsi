import cv2
import numpy as np
import serial
import struct
import time
from collections import deque
import csv
from datetime import datetime

# ── Koneksi Serial ────────────────────────────────────────────────────────────
Arduino = serial.Serial('COM14', baudrate=115200, timeout=0)
Arduino.flushInput()
time.sleep(2)

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FPS, 30)

# Warmup kamera
print("Warming up kamera...")
for _ in range(30):
    cap.read()
time.sleep(1)
print("Kamera siap.")

# ── Parameter Pengujian ────────────────────────────────────────────────────────
TARGET_FPS = 10
CAMERA_FPS = 30
SKIP       = max(1, round(CAMERA_FPS / TARGET_FPS))

# Isi 'y' kalau sesi ini mau disimpan ke CSV, isi 'n' kalau cuma coba-coba/kalibrasi
SIMPAN_CSV = 'y'

# ── Kalibrasi Piksel (satu-satunya sumber kebenaran) ──────────────────────────
# Beam 48 cm (titik A ke G) = 522 px. Ini menggantikan baseline lama
# (583 px / 534 px) yang sudah tidak dipakai lagi.
X_LEFT   = 59     # px, titik A = 0 cm
X_RIGHT  = 581    # px, titik G = 48 cm
BEAM_CM  = 48
BEAM_PX  = X_RIGHT - X_LEFT          # = 522 px, dipakai utk turunan S_PX
S_PX     = BEAM_PX / BEAM_CM         # px/cm, turunan langsung dari kalibrasi di atas

# ── Parameter Model Koreksi Geometri (Subbab 2.6.1) ───────────────────────────
# PENTING: BETA_DEG harus diubah manual sesuai sudut kamera fisik yang
# sedang terpasang SEBELUM program dijalankan (-30, 0, atau +30).
BETA_DEG = 40                      # sudut kemiringan kamera saat ini (derajat)
Z        = 67.3                   # jarak kamera ke beam (cm) -- hasil fitting terbaik
P0       = 320                    # referensi piksel pusat beam (= titik tengah X_LEFT..X_RIGHT)

# F WAJIB diturunkan dari Z * S_PX, BUKAN konstanta terpisah (dulu F=725
# hardcoded independen dari Z & S_PX -> saat beta=0 hasil koreksi TIDAK
# persis sama dengan piksel mentah, selisih sistematis sampai ±0.1 px).
# Dengan F = Z*S_PX, pada beta=0 x_linear == x_pixel_raw persis (terverifikasi).
F        = Z * S_PX

BETA_RAD = np.radians(BETA_DEG)


def correct_position(x_pixel):
    """
    Mengoreksi posisi piksel mentah hasil deteksi menjadi posisi piksel
    linear yang bebas distorsi perspektif akibat kemiringan kamera,
    berdasarkan model geometri pada Subbab 2.6.1.

    Mengembalikan (x_linear, c):
      x_linear : posisi piksel terkoreksi (domain sama dgn piksel mentah),
                 dikirim ke mikrokontroler
      c        : posisi bola relatif pusat beam (cm), untuk keperluan logging
    """
    x_p_prime = x_pixel - P0
    denom = F * np.cos(BETA_RAD) - x_p_prime * np.sin(BETA_RAD)

    # Guard pembagian oleh nilai mendekati nol (seharusnya tidak tercapai
    # pada rentang sudut ±30° dan panjang beam yang digunakan)
    if abs(denom) < 1e-6:
        return float(x_pixel), None

    c = (x_p_prime * Z) / denom
    x_linear = P0 + c * S_PX
    return x_linear, c


set_point = 320
upper = np.array([20, 255, 255])
lower = np.array([5, 120, 120])

x_pixel_raw = 0     # posisi piksel mentah (sebelum koreksi)
x_corrected = 0.0    # posisi piksel setelah koreksi geometri
frame_count = 0
frame_timestamps = deque()

# ── CSV ───────────────────────────────────────────────────────────────────────
simpan_csv = SIMPAN_CSV.strip().lower() == 'y'
csv_file   = None
csv_writer = None

if simpan_csv:
    csv_filename = f"beta{BETA_DEG}_{TARGET_FPS}fps_{datetime.now().strftime('%H%M')}.csv"
    csv_file     = open(csv_filename, 'w', newline='')
    csv_writer   = csv.writer(csv_file)
    csv_writer.writerow(['time_s', 'x_pixel_raw', 'x_corrected', 'c_cm', 'set_point', 'beta_deg'])

t_start = time.perf_counter()
print(f"Sudut kamera (beta): {BETA_DEG} derajat")
print(f"F turunan (Z*S_PX) : {F:.2f} px")
if simpan_csv:
    print(f"Logging ke: {csv_filename}")
else:
    print("CSV tidak disimpan (SIMPAN_CSV='n').")
print("Tekan 'q' untuk berhenti.")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    frame_count += 1
    if frame_count % SKIP != 0:
        continue

    now = time.perf_counter()
    frame_timestamps.append(now)
    while frame_timestamps and (now - frame_timestamps[0]) > 1.0:
        frame_timestamps.popleft()
    fps = len(frame_timestamps)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask    = cv2.inRange(hsv, lower, upper)
    mask    = cv2.erode(mask, None, iterations=2)
    mask    = cv2.dilate(mask, None, iterations=2)
    cnt     = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    ball_detected = False
    if len(cnt) > 0:
        c_contour = max(cnt, key=cv2.contourArea)
        ((cx, cy), radius) = cv2.minEnclosingCircle(c_contour)
        M = cv2.moments(c_contour)
        if M["m00"] != 0 and radius > 15:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            x_pixel_raw = center[0]
            ball_detected = True
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.circle(frame, center, int(radius), (0, 255, 0), 2)

    # ── Koreksi geometri (Subbab 2.6.1) ─────────────────────────────────────
    x_corrected, c_cm = correct_position(x_pixel_raw)

    # ── Catat ke CSV (raw & terkoreksi, untuk keperluan validasi 2.8.1) ────
    if simpan_csv:
        csv_writer.writerow([
            round(now - t_start, 4),
            x_pixel_raw,
            round(x_corrected, 2),
            round(c_cm, 3) if c_cm is not None else '',
            set_point,
            BETA_DEG
        ])

    # ── Kirim posisi TERKOREKSI ke mikrokontroler ───────────────────────────
    packet = struct.pack('<BH', 0xAA, int(round(x_corrected)))
    try:
        Arduino.write(packet)
    except serial.SerialException as e:
        print(f"Serial error: {e}")

    error        = -(set_point - x_corrected)
    status_color = (0, 255, 0) if ball_detected else (0, 0, 255)

    cv2.putText(frame, f"Beta      = {BETA_DEG} deg",        (10, 20),  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
    cv2.putText(frame, f"Set Point = {set_point}",           (10, 40),  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    cv2.putText(frame, f"Raw Pixel = {x_pixel_raw}",         (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)
    cv2.putText(frame, f"Corrected = {x_corrected:.1f}",     (10, 80),  cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)
    cv2.putText(frame, f"Error     = {error:.1f}",           (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    cv2.putText(frame, f"FPS       = {fps} / {TARGET_FPS}",  (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if simpan_csv:
    csv_file.close()
cap.release()
cv2.destroyAllWindows()
Arduino.close()
if simpan_csv:
    print(f"Selesai. {csv_filename} tersimpan.")
else:
    print("Selesai. (CSV tidak disimpan)")