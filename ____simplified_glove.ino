#include "MPU_6050.h"

// enum pentru apel
enum FingerID { INDEX, LITTLE };

MPU_6050 mpuPALM(0x68, 0, "PALM");
MPU_6050 mpuINDX(0x68, 1, "INDEX");
MPU_6050 mpuLITT(0x68, 2, "LITTLE");

// Variabile pentru calibrare
unsigned long loopCount = 0;
const unsigned long RECALIB_EVERY = 100; 

bool calibrated = false;
unsigned long calibStart;
const unsigned long CALIB_DURATION = 2000; // 2 sec
unsigned long calibCount = 0;
double sumRelI = 0;
double sumRel2I = 0;
double sumRelL = 0;
double sumRel2L = 0;

// Praguri (se vor inițializa după calibrare)
float peakThresholdI = 0.0f;
float fallThresholdI = 0.0f;

float peakThresholdL = 0.0f;
float fallThresholdL = 0.0f;

// FSM pentru degete
enum ClickState { IDLE, SPIKE1, WAIT1, SPIKE2, VALIDATE };
ClickState  stateIndex = IDLE, stateLittle = IDLE;
// timestamps and angles
unsigned long t1Index, t2Index, t1Little, t2Little;
float a1Index, a2Index, a1Little, a2Little;
// your timing and angle thresholds
const unsigned long MAX_TIME1 = 750;   // ms between spike1→spike2
const unsigned long MAX_TIME2 = 1500;   // total ms allowed
const float MIN_ANGLE  = 5.0f; // degrees

// Filtru EWMA pentru accel relativ
float filteredRelIndex = 0;
float filteredRelLittle = 0;
const float SMOOTHING = 0.2f;  // între 0.1 și 0.3

// prevent tiny float errors from pushing us outside [–1…1]
static inline float clamp1(float x) {
  return x < -1 ? -1 : (x > +1 ? +1 : x);
}

void setup() {
  Wire.begin(21, 22, 50000);
  Serial.begin(115200);

  // Inițializare și calibrare giroscop + poziție neutră pentru palmă
  mpuPALM.initSensor(300, 300, 5000);
  mpuPALM.printCalibrationData();
  // După aceasta, minPitch/maxPitch și minRoll/maxRoll pentru palmă sunt setate

  // Inițializare degete (fără ROM)
  mpuINDX.initSensor(0, 0, 0);
  mpuLITT.initSensor(0, 0, 0);

  // Pornim calibrarea pentru praguri
  calibStart = millis();
}

void loop() {
  unsigned long now = millis();

  // 1) Citește accelerațiile
  mpuPALM.readAccel();
  mpuINDX.readAccel();
  mpuLITT.readAccel();

  // 2) Dacă suntem încă în calibrare:
  if (!calibrated && now - calibStart < CALIB_DURATION) {
    // index
    float dxI = mpuINDX.ax_g - mpuPALM.ax_g;
    float dyI = mpuINDX.ay_g - mpuPALM.ay_g;
    float dzI = mpuINDX.az_g - mpuPALM.az_g;
    float rI = sqrt(dxI*dxI + dyI*dyI + dzI*dzI);

    float dxL = mpuLITT.ax_g - mpuPALM.ax_g;
    float dyL = mpuLITT.ay_g - mpuPALM.ay_g;
    float dzL = mpuLITT.az_g - mpuPALM.az_g;
    float rL = sqrt(dxL*dxL + dyL*dyL + dzL*dzL);

    calibCount++;
    sumRelI  += rI;
    sumRel2I += rI * rI;

    sumRelL  += rL;
    sumRel2L += rL * rL;

    return;
  }

  // post calibrare
  if (!calibrated) {
    float meanI = sumRelI  / calibCount;
    float varianceI = (sumRel2I / calibCount) - meanI * meanI;
    float stdDevI = sqrt(varianceI);

    float meanL = sumRelL  / calibCount;
    float varianceL = (sumRel2L / calibCount) - meanL * meanL;
    float stdDevL = sqrt(varianceL);

    peakThresholdI = meanI + 4.0f * stdDevI;
    fallThresholdI = meanI + 1.0f * stdDevI;

    peakThresholdL = meanL + 4.0f * stdDevL;
    fallThresholdL = meanL + 1.0f * stdDevL;

    Serial.printf("[INDX] Prag spike: %.3f  |  Prag cădere: %.3f\n",
                  peakThresholdI, fallThresholdI);

    Serial.printf("[LITT] Prag spike: %.3f  |  Prag cădere: %.3f\n",
                  peakThresholdL, fallThresholdL);
    calibrated = true;
  }

  // De aici începe detecția efectivă...
  detectClicks();
}

void detectClicks() {
  unsigned long now = millis();

  // 0) DEBUG — print both relAccel & angle vs their thresholds
  float relI = calcRelAccel(mpuINDX);
  float relL = calcRelAccel(mpuLITT);
  float angI = calcRelAngle(mpuINDX);
  float angL = calcRelAngle(mpuLITT);
    Serial.printf(
    "[DBG] INDEX relA=%.3f (peak=%.3f, fall=%.3f)  "
    "LITTLE relA=%.3f (peak=%.3f, fall=%.3f)  "
    "angI=%.2f  angL=%.2f\n",
    relI, peakThresholdI, fallThresholdI,
    relL, peakThresholdL, fallThresholdL,
    angI, angL
  );

  // 1) Now run your FSMs as before
  fsmStep(INDEX,  relI,  angI, now, peakThresholdI, fallThresholdI);
  fsmStep(LITTLE, relL,  angL, now, peakThresholdL, fallThresholdL);
}



float calcRelAccel(MPU_6050 &finger) {
  float dx = finger.ax_g - mpuPALM.ax_g;
  float dy = finger.ay_g - mpuPALM.ay_g;
  float dz = finger.az_g - mpuPALM.az_g;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

float calcRelAngle(MPU_6050 &finger) {
  // Make sure both sensors have fresh accel→ax_g,ay_g,az_g
  mpuPALM.readAccel();
  finger.readAccel();

  // 1) compute each magnitude
  float magP = sqrt(mpuPALM.ax_g*mpuPALM.ax_g
                  + mpuPALM.ay_g*mpuPALM.ay_g
                  + mpuPALM.az_g*mpuPALM.az_g);
  float magF = sqrt(finger.ax_g*finger.ax_g
                  + finger.ay_g*finger.ay_g
                  + finger.az_g*finger.az_g);

  // 2) compute their dot
  float dot =  mpuPALM.ax_g * finger.ax_g
             + mpuPALM.ay_g * finger.ay_g
             + mpuPALM.az_g * finger.az_g;

  // 3) normalize & clamp
  float cosA = clamp1(dot / (magP * magF));

  // 4) →degrees
  return acos(cosA) * 180.0f / PI;
}

void fsmStep(FingerID id,
             float relAccel, float angle,
             unsigned long now,
             float peakThr, float fallThr)
{
  // pick the right state and storage
  ClickState &st  = (id==INDEX ? stateIndex  : stateLittle);
  unsigned long &t1 = (id==INDEX ? t1Index : t1Little);
  unsigned long &t2 = (id==INDEX ? t2Index : t2Little);
  float        &a1 = (id==INDEX ? a1Index  : a1Little);
  float        &a2 = (id==INDEX ? a2Index  : a2Little);
  const char   *name = (id==INDEX ? "INDEX" : "LITTLE");

  switch(st) {
    case IDLE:
      if (relAccel > peakThr) {
        st = SPIKE1;
        Serial.printf("[%s] → SPIKE1 (relA=%.3f > %.3f)\n", name, relAccel, peakThr);
      }
      break;

    case SPIKE1:
      if (relAccel < fallThr) {
        t1 = now;  a1 = angle;
        st = WAIT1;
        Serial.printf("[%s] → FALL to WAIT1: t1=%lu, a1=%.2f°\n", name, t1, a1);
      }
      break;

    case WAIT1:
      if (now - t1 > MAX_TIME1) {
        Serial.printf("[%s] WAIT1 timeout (dt=%lums) → IDLE\n", name, now - t1);
        st = IDLE;
      }
      else if (relAccel > peakThr) {
        st = SPIKE2;
        Serial.printf("[%s] → SPIKE2 (relA=%.3f > %.3f)\n", name, relAccel, peakThr);
      }
      break;

    case SPIKE2:
      if (relAccel < fallThr) {
        t2 = now;  a2 = angle;
        st = VALIDATE;
        Serial.printf("[%s] → FALL2 to VALIDATE: t2=%lu, a2=%.2f°\n", name, t2, a2);
      }
      break;

    case VALIDATE:
      {
        unsigned long dt = t2 - t1;
        float dA = fabs(a2 - a1);
        Serial.printf("[%s] VALIDATE: dt=%lums, dA=%.2f°\n", name, dt, dA);
        if (dt < MAX_TIME2 && dA > MIN_ANGLE) {
          Serial.printf("✔ %s click @ %lu\n", name, now);
          delay(2000);
        }
        st = IDLE;
      }
      break;
  }
}



void reportPalmPosition() {
  float roll  = mpuPALM.getRoll();
  float pitch = mpuPALM.getPitch();

  float midRoll  = (mpuPALM.getMinRoll()  + mpuPALM.getMaxRoll())  * 0.5f;
  float midPitch = (mpuPALM.getMinPitch() + mpuPALM.getMaxPitch()) * 0.5f;

  if (roll > midRoll)     Serial.println("Palm: supinatie");
  else                    Serial.println("Palm: pronatie");

  if (pitch > midPitch)   Serial.println("Palm: flexie");
  else                    Serial.println("Palm: extensie");
}