// ESP32 Double-Click Detection — Multicore Version
// =====================================================
// **Overview**:
// - Core 0 (`taskSensor`): Reads three MPU-6050 accelerometers, computes
//   relative acceleration (finger vs. palm) and dot-product/squared-magnitude
//   data for angle tests at ~200 Hz, and publishes to a shared struct.
// - Core 1 (`loop`): Retrieves each sample, updates adaptive thresholds (EWMA),
//   and runs two FSMs (INDEX and LITTLE) to detect double-click gestures
//   based on accel spikes and measured angular bounce-back.

#include <Wire.h>
#include "MPU_6050.h"

//–– CONFIGURATION ––
const int    RECALIB_INTERVAL = 25;     // samples per threshold update
const float  ALPHA_THRESH     = 0.80f;  // EWMA blend factor
const float  MIN_ANGLE        = 35.0f;  // min bounce-back (deg)
const unsigned long MAX_DT1   = 1000;   // ms max between spike1→spike2
const unsigned long MAX_DT2   = 2000;   // ms max total window

//–– LITTLE-FINGER SENSITIVITY CLAMPS ––
const float LIT_MIN_PEAK = 0.50f;  // never require >0.5g spike
const float LIT_MIN_FALL = 0.30f;  // never require <0.3g fall

//–– MPU INSTANCES ––
MPU_6050 mpuPALM(0x68, 0, "PALM");
MPU_6050 mpuINDX(0x68, 1, "INDEX");
MPU_6050 mpuLITT(0x68, 2, "LITTLE");

//–– CLICK-FSM STATE ––
enum ClickState { IDLE, SPIKE1, WAIT1, SPIKE2, VALIDATE };
ClickState stateI = IDLE, stateL = IDLE;

// Adaptive thresholds
float peakI = 0, fallI = 0;
float peakL = 0, fallL = 0;

// Buffers for recalibration
float bufRelI[RECALIB_INTERVAL];
float bufRelL[RECALIB_INTERVAL];
int   bufPos = 0;

// Timestamps/angles for FSM
unsigned long t1I, t2I, t1L, t2L;
float a1I, a2I, a1L, a2L;

//–– SHARED SAMPLE STRUCT ––
struct Sample {
  float relI, relL;           // relative accel
  float dotI, mag2PI, mag2FI; // index angle test
  float dotL, mag2PL, mag2FL; // little angle test
  unsigned long ts;
};
static Sample latestSample;
portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;

//–– UTILITY ––
static inline float calcAngleDeg(float dot, float m2p, float m2f) {
  float cosA = dot / sqrt(m2p * m2f);
  cosA = constrain(cosA, -1.0f, 1.0f);
  return acos(cosA) * 180.0f/PI;
}

// forward sensor task
void taskSensor(void* pv);

//–– SETUP ––
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  mpuPALM.initSensor(300,300,5000);
  mpuINDX.initSensor(0,0,0);
  mpuLITT.initSensor(0,0,0);

  for (int i = 0; i < RECALIB_INTERVAL; i++)
    bufRelI[i] = bufRelL[i] = 0;

  xTaskCreatePinnedToCore(
    taskSensor, "Sensor", 4096, nullptr, 1, nullptr, 0
  );
}

//–– MAIN LOOP (core 1) ––
void loop() {
  // 1) grab sample
  Sample s;
  portENTER_CRITICAL(&sampleMux);
    memcpy(&s, &latestSample, sizeof(s));
  portEXIT_CRITICAL(&sampleMux);

  // 2) buffer rel‐accel
  bufRelI[bufPos] = s.relI;
  bufRelL[bufPos] = s.relL;
  bufPos = (bufPos + 1) % RECALIB_INTERVAL;

  // 3) update thresholds
  static int cnt = 0;
  static bool warmed = false;
  if (!warmed) {
    if (++cnt >= RECALIB_INTERVAL) { warmed = true; cnt = 0; }
  } else if (++cnt >= RECALIB_INTERVAL) {
    // sums & variances
    float sumI=0, sqI=0, sumL=0, sqL=0;
    for (int i = 0; i < RECALIB_INTERVAL; i++) {
      sumI += bufRelI[i]; sqI += bufRelI[i]*bufRelI[i];
      sumL += bufRelL[i]; sqL += bufRelL[i]*bufRelL[i];
    }
    float mI = sumI/RECALIB_INTERVAL;
    float vI = max(0.0f, sqI/RECALIB_INTERVAL - mI*mI);
    float sI = sqrt(vI);
    float mL = sumL/RECALIB_INTERVAL;
    float vL = max(0.0f, sqL/RECALIB_INTERVAL - mL*mL);
    float sL = sqrt(vL);

    // compute new raw thresholds
    float newPI = mI + 1.1f*sI, newFI = mI + 0.1f*sI;
    float newPL = mL + 0.8f*sL, newFL = mL + 0.1f*sL;

    // EWMA blend
    peakI = ALPHA_THRESH*peakI + (1-ALPHA_THRESH)*newPI;
    fallI = ALPHA_THRESH*fallI + (1-ALPHA_THRESH)*newFI;

    peakL = ALPHA_THRESH*peakL + (1-ALPHA_THRESH)*newPL;
    fallL = ALPHA_THRESH*fallL + (1-ALPHA_THRESH)*newFL;

    // clamp little‐finger thresholds to minimums
    peakL = max(peakL, LIT_MIN_PEAK);
    fallL = max(fallL, LIT_MIN_FALL);

    Serial.printf("New thresholds -> I(%.2f,%.2f) L(%.2f,%.2f)\n",
                  peakI, fallI, peakL, fallL);
    cnt = 0;
  }

  // 4) compute angles
  float angI = calcAngleDeg(s.dotI, s.mag2PI, s.mag2FI);
  float angL = calcAngleDeg(s.dotL, s.mag2PL, s.mag2FL);

  // 5) run FSMs
  bool indexDominant = (s.relI > s.relL);

  if (indexDominant) {
    // only INDEX can advance - reset LITTLE outright
    fsmStep(stateI, s.relI, angI,
            t1I, t2I, a1I, a2I,
            peakI, fallI, "INDEX");
    stateL = IDLE;
  }
  else {
    // only LITTLE can advance - reset INDEX outright
    fsmStep(stateL, s.relL, angL,
            t1L, t2L, a1L, a2L,
            peakL, fallL, "LITTLE");
    stateI = IDLE;
  }

  delay(1);
}

//–– SENSOR TASK (core 0) ––
void taskSensor(void* pv) {
  Sample tmp;
  while (1) {
    mpuPALM.readAccel();
    mpuINDX.readAccel();
    mpuLITT.readAccel();

    tmp.relI = sqrt(sq(mpuINDX.ax_g - mpuPALM.ax_g)
                  + sq(mpuINDX.ay_g - mpuPALM.ay_g)
                  + sq(mpuINDX.az_g - mpuPALM.az_g));
    tmp.relL = sqrt(sq(mpuLITT.ax_g - mpuPALM.ax_g)
                  + sq(mpuLITT.ay_g - mpuPALM.ay_g)
                  + sq(mpuLITT.az_g - mpuPALM.az_g));

    tmp.dotI   = mpuPALM.ax_g*mpuINDX.ax_g
               + mpuPALM.ay_g*mpuINDX.ay_g
               + mpuPALM.az_g*mpuINDX.az_g;
    tmp.mag2PI = sq(mpuPALM.ax_g) + sq(mpuPALM.ay_g) + sq(mpuPALM.az_g);
    tmp.mag2FI = sq(mpuINDX.ax_g) + sq(mpuINDX.ay_g) + sq(mpuINDX.az_g);

    tmp.dotL   = mpuPALM.ax_g*mpuLITT.ax_g
               + mpuPALM.ay_g*mpuLITT.ay_g
               + mpuPALM.az_g*mpuLITT.az_g;
    tmp.mag2PL = tmp.mag2PI;
    tmp.mag2FL = sq(mpuLITT.ax_g) + sq(mpuLITT.ay_g) + sq(mpuLITT.az_g);

    tmp.ts = millis();

    portENTER_CRITICAL(&sampleMux);
      memcpy(&latestSample, &tmp, sizeof(tmp));
    portEXIT_CRITICAL(&sampleMux);

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

//–– CLICK FSM FUNCTION ––
void fsmStep(ClickState& st,
             float rel,
             float angle,
             unsigned long& t1,
             unsigned long& t2,
             float& a1,
             float& a2,
             float peak,
             float fall,
             const char* name) {
  unsigned long now = millis();
  switch(st) {
    case IDLE:
      if (rel > peak) {
        st = SPIKE1;  Serial.printf("[%s] SPIKE1\n", name);
      }
      break;
    case SPIKE1:
      if (rel < fall) {
        t1 = now; a1 = angle; st = WAIT1;
        Serial.printf("[%s] FALL→WAIT1\n", name);
      }
      break;
    case WAIT1:
      if (now - t1 > MAX_DT1) st = IDLE;
      else if (rel > peak) {
        st = SPIKE2; Serial.printf("[%s] SPIKE2\n", name);
      }
      break;
    case SPIKE2:
      if (rel < fall) {
        t2 = now; a2 = angle; st = VALIDATE;
        Serial.printf("[%s] FALL2→VALIDATE\n", name);
      }
      break;
    case VALIDATE: {
      unsigned long dt = t2 - t1;
      float da = fabs(a2 - a1);
      Serial.printf("[%s] VALIDATE dt=%lums da=%.2f°\n", name, dt, da);
      if (dt < MAX_DT2 && da > MIN_ANGLE) {
        Serial.printf("✔ %s click\n", name);
        delay(500);
      }
      st = IDLE;
    } break;
  }
}
