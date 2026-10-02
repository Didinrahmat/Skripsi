#include <ESP32Servo.h>

// ── Servo & Konstanta ────────────────────────────────────────────────────────
Servo s;
const int SERVO_PIN   = 4;
const int DC          = 1380;
const int SERVO_LIMIT = 450;

// const int DC          = 1420;

// ── PID Parameter ────────────────────────────────────────────────────────────
const int SET_POINT = 320;
float Kp    = 2.25;
float Ki    = 0.5;
float Kd    = 4.6;

float alpha = 0.5;



// ini tuningan terbaru 23/07/2026
// float Kp    = 2.25;
// float Ki    = 0.5;
// float Kd    = 4.6;
// float alpha = 0.5;

// float Kp    = 2.2;
// float Ki    = 0.5;
// float Kd    = 4.5;
// float alpha = 0.5;

// ── State Shared (ISR <-> loop) ───────────────────────────────────────────────
volatile int  lastBallPos      = SET_POINT;  // UBAH: dari 0 ke SET_POINT
volatile bool dataEverReceived = false;       // TAMBAH: flag data pertama

// ── State Internal PID (hanya ISR) ───────────────────────────────────────────
float errPrev    = 0.0;
float integral   = 0.0;
float dFiltered  = 0.0;
unsigned long timePrevUs = 0;

// ── Timer ─────────────────────────────────────────────────────────────────────
hw_timer_t* pidTimer = NULL;

void IRAM_ATTR onPidTimer() {
  // TAMBAH: tahan PID sampai data pertama dari Python tiba
  if (!dataEverReceived) {
    s.writeMicroseconds(DC);
    return;
  }

  int ballPos = lastBallPos;

  unsigned long now = micros();
  float dt = (now - timePrevUs) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
  timePrevUs = now;

  float err  = -(float)(SET_POINT - ballPos);
  float prop = err * Kp;

  if (err > -50.0f && err < 50.0f) {
    integral += err * Ki * dt;
  } else {
    integral = 0.0f;
  }
  integral = constrain(integral, -150.0f, 150.0f);

  float dRaw  = (err - errPrev) / dt;
  dFiltered   = alpha * dRaw + (1.0f - alpha) * dFiltered;
  float d     = (err > -5.0f && err < 5.0f) ? 0.0f : dFiltered * Kd;

  errPrev = err;

  float u      = prop + d + integral;
  int servoPos = constrain((int)(DC + u), DC - SERVO_LIMIT, DC + SERVO_LIMIT);
  s.writeMicroseconds(servoPos);
}

// ── Parser Serial (state machine) ─────────────────────────────────────────────
enum ParseState { WAIT_HEADER, READ_LOW, READ_HIGH };
ParseState parseState = WAIT_HEADER;
uint8_t rawLow = 0;

void parseSerial() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch (parseState) {
      case WAIT_HEADER:
        if (b == 0xAA) parseState = READ_LOW;
        break;
      case READ_LOW:
        rawLow     = b;
        parseState = READ_HIGH;
        break;
      case READ_HIGH:
        lastBallPos      = (int)(rawLow | ((uint16_t)b << 8));
        dataEverReceived = true;  // TAMBAH: set flag saat paket pertama diterima
        parseState       = WAIT_HEADER;
        break;
    }
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  s.attach(SERVO_PIN, 1000, 2000);
  s.writeMicroseconds(DC);

  delay(3000);
  timePrevUs = micros();

  pidTimer = timerBegin(1000000);
  timerAttachInterrupt(pidTimer, &onPidTimer);
  timerAlarm(pidTimer, 10000, true, 0);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  parseSerial();
}