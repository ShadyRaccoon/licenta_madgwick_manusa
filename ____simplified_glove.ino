#include <Wire.h>
#include "MPU_6050.h"

//–– CONFIGURATION ––
unsigned long palmSpikeTs = 0;
const unsigned long PALM_COOLDOWN = 1000;  // ms

float palmEWMA = 0.0f;
const float PALM_ALPHA     = 0.1f;   // small alpha => slow, smooth palm signal
const float PALM_THRESHOLD = 1.10f;

const int PALM_BUF_SZ       = 100;
const float PALM_STEADY_DEG = 0.01f;   // allow ±0.1g around the avg
float palmBuf[PALM_BUF_SZ];
int palmBufPos      = 0;
bool palmBufWarm    = false;

const int    RECALIB_INTERVAL = 200;    // not used for dual-EWMA, but kept for reference
const float  ALPHA_FAST       = 0.2f;   // EWMA α for fast tracking (0.0–1.0)
const float  LIT_MIN_PEAK     = 0.15f;  // little finger floor for peak
const float  LIT_MIN_FALL     = 0.08f;  // little finger floor for fall
const float  MIN_ANGLE        = 20.0f;  // degrees for bounce-back
const unsigned long MAX_DT1   = 800;    // ms between spike1 and spike2
const unsigned long MAX_DT2   = 1200;   // ms total for both spikes
const unsigned long PHASE_MS  = 50;     // how long before flipping EWMA phase

//–– MPU INSTANCES ––
MPU_6050 mpuPALM(0x68,0,"PALM"),
        mpuINDX(0x68,1,"INDEX"),
        mpuLITT(0x68,3,"LITTLE");

//-- PALM GESTURES --
enum Direction { FRONT, BACK, LEFT, RIGHT, HOVER, NEUTRAL };

float FRONT_TH, BACK_TH, LEFT_TH, RIGHT_TH;

Direction direction = NEUTRAL;
Direction lastDirection = NEUTRAL;

void setDirectionThresholds(MPU_6050 &mpu);
void updateDirection(MPU_6050 &mpu);
void printDirection();

//–– CLICK FSM ––
enum ClickState { IDLE, SPIKE1, WAIT1, SPIKE2, VALIDATE };
ClickState stI=IDLE, stL=IDLE;
unsigned long t1I,t2I,t1L,t2L;
float a1I,a2I,a1L,a2L;

const unsigned long DOUBLE_CLICK_WINDOW = 1500; //ms
const unsigned long CLICK_COOLDOWN = 400;

unsigned long lastClickTimeI = 0;
uint8_t clickCountI          = 0;

unsigned long lastClickTimeL = 0;
uint8_t clickCountL          = 0; 

//–– INTER-CORE SHARED SAMPLE ––
struct Sample {
  float palmMag;
  float relI, relL;
  float dotI, mag2PI, mag2FI;
  float dotL, mag2PL, mag2FL;
  unsigned long ts;
};
static Sample latestSample;
portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;

//–– DUAL-EWMA STATE ––
const float FLT_MAX = 1e6f;
// Index finger
float ewma1_I=0, ewma2_I=0;
float ewma1_min_I=FLT_MAX, ewma1_max_I=-FLT_MAX;
float ewma2_min_I=FLT_MAX, ewma2_max_I=-FLT_MAX;
// Little finger
float ewma1_L=0, ewma2_L=0;
float ewma1_min_L=FLT_MAX, ewma1_max_L=-FLT_MAX;
float ewma2_min_L=FLT_MAX, ewma2_max_L=-FLT_MAX;
// Phase toggling
bool phase = false;                     
unsigned long lastSwitch = 0;

//–– UTILITY ––
static inline float calcAngleDeg(float dot, float m2p, float m2f) {
  float c = dot / sqrt(m2p * m2f);
  c = constrain(c, -1.0f, 1.0f);
  return acos(c) * 180.0f/PI;
}

//–– SENSOR TASK (unchanged) ––
void taskSensor(void* pv) {
  Sample tmp;
  while (true) {
    mpuPALM.readAccel(); mpuINDX.readAccel(); mpuLITT.readAccel();
    tmp.palmMag = sqrt(
      sq(mpuPALM.ax_g) +
      sq(mpuPALM.ay_g) +
      sq(mpuPALM.az_g)
    );
    tmp.relI = sqrt(sq(mpuINDX.ax_g-mpuPALM.ax_g)
                  + sq(mpuINDX.ay_g-mpuPALM.ay_g)
                  + sq(mpuINDX.az_g-mpuPALM.az_g));
    tmp.relL = sqrt(sq(mpuLITT.ax_g-mpuPALM.ax_g)
                  + sq(mpuLITT.ay_g-mpuPALM.ay_g)
                  + sq(mpuLITT.az_g-mpuPALM.az_g));
    tmp.dotI   = mpuPALM.ax_g*mpuINDX.ax_g
               + mpuPALM.ay_g*mpuINDX.ay_g
               + mpuPALM.az_g*mpuINDX.az_g;
    tmp.mag2PI = sq(mpuPALM.ax_g)+sq(mpuPALM.ay_g)+sq(mpuPALM.az_g);
    tmp.mag2FI = sq(mpuINDX.ax_g)+sq(mpuINDX.ay_g)+sq(mpuINDX.az_g);
    tmp.dotL   = mpuPALM.ax_g*mpuLITT.ax_g
               + mpuPALM.ay_g*mpuLITT.ay_g
               + mpuPALM.az_g*mpuLITT.az_g;
    tmp.mag2PL = tmp.mag2PI;
    tmp.mag2FL = sq(mpuLITT.ax_g)+sq(mpuLITT.ay_g)+sq(mpuLITT.az_g);
    tmp.ts     = millis();
    portENTER_CRITICAL(&sampleMux);
      memcpy(&latestSample, &tmp, sizeof(tmp));
    portEXIT_CRITICAL(&sampleMux);
    updateDirection(mpuPALM);
    if(lastDirection != direction)
      printDirection();
    lastDirection = direction;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

//–– CLICK FSM FUNCTION & HELPER––
void clickCounter(bool isIndex){
  unsigned long now = millis();

  unsigned long &lastClickTime = isIndex ? lastClickTimeI : lastClickTimeL;
  uint8_t &clickCount = isIndex? clickCountI : clickCountL;

  if(clickCount == 1 && (now - lastClickTime < CLICK_COOLDOWN)) return;

  if(clickCount == 1 && (now - lastClickTime < DOUBLE_CLICK_WINDOW)){
    Serial.printf("✔ %s DOUBLE-CLICK\n", isIndex?"INDEX":"LITTLE");
    delay(1000);
    clickCount = 0;      // reset
    lastClickTime = 0;
  } else {
    clickCount = 1;
    lastClickTime = now;
  }
}

void fsmStep(ClickState& st,
             float rel,
             float palmMag,
             float angle,
             unsigned long& t1,
             unsigned long& t2,
             float& a1,
             float& a2,
             float peak,
             float fall,
             const char* name) {
  unsigned long now = millis();
  switch (st) {
    case IDLE:
      if (rel > peak) {
        st = SPIKE1; 
        // Serial.printf("[%s] SPIKE1\n", name);
      }
      break;
    case SPIKE1:
      if (rel < fall) {
        t1 = now; a1 = angle; st = WAIT1;
      //  Serial.printf("[%s] FALL→WAIT1\n", name);
      }
      break;
    case WAIT1:
      if (now - t1 > MAX_DT1) st = IDLE;
      else if (rel > peak) {
        st = SPIKE2;
      //  Serial.printf("[%s] SPIKE2\n", name);
      }
      break;
    case SPIKE2:
      if (rel < fall) {
        t2 = now; a2 = angle; st = VALIDATE;
        //Serial.printf("[%s] FALL2→VALIDATE\n", name);
      }
      break;
    case VALIDATE: {
      unsigned long dt = t2 - t1;
      float da = fabs(a2 - a1);
      //Serial.printf("[%s] VALIDATE dt=%lums da=%.2f°\n", name, dt, da);
      if (dt < MAX_DT2 && da > MIN_ANGLE){
        //Serial.printf("[%s] VALIDATE dt=%lums da=%.2f° rel=%.2fg palm=%.2fg diff=%.2fg\n",
        //        name, dt, da, rel, palmMag, palmMag-rel);
        clickCounter(strcmp(name, "INDEX") == 0);
      }
      st = IDLE;
    } break;
  }
}

//-- DIRECTION FUNCTION --
void checkDirection(){
  
}

//–– SETUP ––
void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);
  mpuPALM.initSensor(300,300,10000);
  mpuINDX.initSensor(0,0,0);
  mpuLITT.initSensor(0,0,0);
  setDirectionThresholds(mpuPALM);
  xTaskCreatePinnedToCore(
    taskSensor, 
    "Sensor", 
    4096, 
    nullptr, 
    1, 
    nullptr, 
    0
  );
  lastSwitch = millis();
}

//–– MAIN LOOP (core 1) ––
void loop() {
  // 1) Grab latest preprocessed sample
  Sample s;
  portENTER_CRITICAL(&sampleMux);
    memcpy(&s, &latestSample, sizeof(s));
  portEXIT_CRITICAL(&sampleMux);
  float rawPalm = s.palmMag;

  // 2) Fill steady‐hand ring buffer
  palmBuf[palmBufPos] = rawPalm;
  if (++palmBufPos >= PALM_BUF_SZ) {
    palmBufPos = 0;
    palmBufWarm = true;
  }

  // 3) Compute steady‐hand mean
  float palmMean = rawPalm;
  if (palmBufWarm) {
    palmMean = 0;
    for (int i = 0; i < PALM_BUF_SZ; i++) palmMean += palmBuf[i];
    palmMean /= PALM_BUF_SZ;
  }

  // 4) Update slow Palm EWMA
  palmEWMA = PALM_ALPHA * palmEWMA + (1 - PALM_ALPHA) * rawPalm;
  //Serial.printf("PALM raw=%.2fg  EWMA=%.2fg  mean=%.2fg\n",
  //              rawPalm, palmEWMA, palmMean);

  // 5) Gate if palm is spiking or unsteady
  if (palmEWMA > PALM_THRESHOLD || fabs(rawPalm - palmMean) > PALM_STEADY_DEG) {
    palmSpikeTs = millis();
    palmBufWarm = false;
    palmBufPos  = 0;
    stI = stL = IDLE;
    return;
  }

  if (millis() - palmSpikeTs < PALM_COOLDOWN) {
    return;
  }

  // 2) Compute rel-accels
  float relI = s.relI;
  float relL = s.relL;

  // 3) Select which EWMA slot to update this phase
  float &eI    = phase ? ewma2_I    : ewma1_I;
  float &minI  = phase ? ewma2_min_I : ewma1_min_I;
  float &maxI  = phase ? ewma2_max_I : ewma1_max_I;
  float &eL    = phase ? ewma2_L    : ewma1_L;
  float &minL  = phase ? ewma2_min_L : ewma1_min_L;
  float &maxL  = phase ? ewma2_max_L : ewma1_max_L;

  // 4) Update EWMA & extrema for this slot
  eI   = ALPHA_FAST * eI + (1 - ALPHA_FAST) * relI;
  minI = min(minI, eI);
  maxI = max(maxI, eI);

  eL   = ALPHA_FAST * eL + (1 - ALPHA_FAST) * relL;
  minL = min(minL, eL);
  maxL = max(maxL, eL);

  // 5) After PHASE_MS, flip phase and reset the newly inactive slot
  if (millis() - lastSwitch >= PHASE_MS) {
    phase = !phase;
    lastSwitch = millis();
    if (phase) {
      ewma1_min_I = FLT_MAX;  ewma1_max_I = -FLT_MAX;
      ewma1_min_L = FLT_MAX;  ewma1_max_L = -FLT_MAX;
    } else {
      ewma2_min_I = FLT_MAX;  ewma2_max_I = -FLT_MAX;
      ewma2_min_L = FLT_MAX;  ewma2_max_L = -FLT_MAX;
    }
  }

  // 6) Once both slots have been touched at least once, derive thresholds
  if (lastSwitch > 0) {
    float peakI = max(ewma1_max_I, ewma2_max_I);
    float fallI = min(ewma1_min_I, ewma2_min_I);
    float peakL = max(ewma1_max_L, ewma2_max_L);
    float fallL = min(ewma1_min_L, ewma2_min_L);
    // clamp little-finger floors
    peakL = max(peakL, LIT_MIN_PEAK);
    fallL = max(fallL, LIT_MIN_FALL);

    // 7) Compute angles
    float angI = calcAngleDeg(s.dotI, s.mag2PI, s.mag2FI);
    float angL = calcAngleDeg(s.dotL, s.mag2PL, s.mag2FL);

    // 8) Run the FSMs
    fsmStep(stI, s.relI, s.palmMag, angI, t1I, t2I, a1I, a2I, peakI, fallI, "INDEX");
    fsmStep(stL, s.relL, s.palmMag, angL, t1L, t2L, a1L, a2L, peakL, fallL, "LITTLE");

  }

  // 9) Yield to maintain ~200Hz
  delay(5);
}

// -- PALM GESTURE FUNCTIONS --
void setDirectionThresholds(MPU_6050 &mpu){
  FRONT_TH = (mpu.getMinAX() + mpu.getNeutralAX()) / 2;
  BACK_TH  = (mpu.getMaxAX() + mpu.getNeutralAX()) / 2;
  LEFT_TH  = (mpu.getMinAY() + mpu.getNeutralAY()) / 2;
  RIGHT_TH = (mpu.getMaxAY() + mpu.getNeutralAY()) / 2;
}

void updateDirection(MPU_6050 &mpu){
  uint8_t front = (mpu.ay_g < FRONT_TH) ? 1 : 0; //flexed => negative roll
  uint8_t back  = (mpu.ay_g > BACK_TH) ? 1 : 0;
  uint8_t left  = (mpu.ax_g < LEFT_TH) ? 1 : 0;
  uint8_t right = (mpu.ax_g > RIGHT_TH) ? 1 : 0;

  if(front + back + left + right != 1 && front + back + left + right != 0) return;
  if(front + back + left + right == 0) direction = HOVER;

  direction = (front == 1) ? FRONT : direction;
  direction = (back == 1) ? BACK : direction;
  direction = (left == 1) ? LEFT : direction;
  direction = (right == 1) ? RIGHT : direction;
}

void printDirection(){
  switch (direction){
    case FRONT:
      Serial.println("DIRECTION: FRONT");
      break;
    case BACK:
      Serial.println("DIRECTION: BACK");
      break;
    case LEFT:
      Serial.println("DIRECTION: LEFT");
      break;
    case RIGHT:
      Serial.println("DIRECTION: RIGHT");
      break;
    default:
      Serial.println("HOVERING");
      break;
  }
}