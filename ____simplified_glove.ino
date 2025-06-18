#include "MPU_6050.h"

// enum pentru apel
enum FingerID { INDEX, LITTLE };

MPU_6050 mpuPALM(0x68, 0, "PALM");
MPU_6050 mpuINDX(0x68, 1, "INDEX");
MPU_6050 mpuLITT(0x68, 2, "LITTLE");

// Variabile pentru calibrare
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
enum State { IDLE, SPIKE1, WAIT1, SPIKE2, VALIDATE };
State stateIndex = IDLE, stateLittle = IDLE;
unsigned long t1Index, t2Index, t1Little, t2Little;
float angle1Index, angle2Index, angle1Little, angle2Little;

// Parametri FSM
const unsigned long MAX_TIME1 = 300;   // 300 ms
const unsigned long MAX_TIME2 = 800;   // 800 ms
const float MIN_ANGLE = 5.0f;          // 5 grade

// Filtru EWMA pentru accel relativ
float filteredRelIndex = 0;
float filteredRelLittle = 0;
const float SMOOTHING = 0.2f;  // între 0.1 și 0.3

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

    peakThresholdI = meanI + 3.0f * stdDevI;      // detectăm doar 1% din zgomot
    fallThresholdI = 0.6f * peakThresholdI;      // prag de "cădere"

    peakThresholdL = meanL + 3.0f * stdDevL;      // detectăm doar 1% din zgomot
    fallThresholdL = 0.6f * peakThresholdL;      // prag de "cădere"

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

  // 1) recalculează relAccel pentru fiecare deget
  float relIndex  = calcRelAccel(mpuINDX);
  float relLittle = calcRelAccel(mpuLITT);

  // 2) EWMA smoothing
  filteredRelIndex  += SMOOTHING * (relIndex  - filteredRelIndex);
  filteredRelLittle += SMOOTHING * (relLittle - filteredRelLittle);

  // 3) relAngle pentru fiecare
  float angleIndex  = calcRelAngle(mpuINDX);
  float angleLittle = calcRelAngle(mpuLITT);

  // 4) FSM
  fsmStep(INDEX,  filteredRelIndex,  angleIndex,
        now, peakThresholdI, fallThresholdI);
  fsmStep(LITTLE, filteredRelLittle, angleLittle,
        now, peakThresholdL, fallThresholdL);

  // 5) detectare poziție palmă
  reportPalmPosition();
}

float calcRelAccel(MPU_6050 &finger) {
  float dx = finger.ax_g - mpuPALM.ax_g;
  float dy = finger.ay_g - mpuPALM.ay_g;
  float dz = finger.az_g - mpuPALM.az_g;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

float calcRelAngle(MPU_6050 &finger) {
  mpuPALM.computeAngles();
  finger.computeAngles();

  float dR = wrap180(finger.getRoll()  - mpuPALM.getRoll());
  float dP = wrap180(finger.getPitch() - mpuPALM.getPitch());

  return sqrt(dR*dR + dP*dP);
}

void fsmStep(FingerID id, float relAccel, float relAngle, unsigned long now, float peakThreshold, float fallThreshold) {
  State &st     = (id==INDEX ? stateIndex  : stateLittle);
  unsigned long &t1 = (id==INDEX ? t1Index : t1Little);
  unsigned long &t2 = (id==INDEX ? t2Index : t2Little);
  float &a1     = (id==INDEX ? angle1Index : angle1Little);
  float &a2     = (id==INDEX ? angle2Index : angle2Little);

  // DEBUG: print current state, accel & angle
  Serial.printf("[%s] state=%d  relA=%.3f (thr=%.3f/%.3f)  relAng=%.2f\n",
    id==INDEX?"IDX":"LTL", st,
    relAccel, peakThreshold, fallThreshold,
    relAngle);

  switch (st) {
    case IDLE:
      if (relAccel > peakThreshold) {
        st = SPIKE1;
        Serial.println("  ✦ SPIKE1");
      }
      break;

    case SPIKE1:
      if (relAccel < fallThreshold) {
        t1 = now; a1 = relAngle;
        st = WAIT1;
        Serial.printf("  ✦ FALL to WAIT1 (t1=%lu, a1=%.2f)\n", t1, a1);
      }
      break;

    case WAIT1:
      if (now - t1 > MAX_TIME1) {
        st = IDLE;
        Serial.println("  ✦ WAIT1 timeout → IDLE");
      } else if (relAccel > peakThreshold) {
        st = SPIKE2;
        Serial.println("  ✦ SPIKE2");
      }
      break;

    case SPIKE2:
      if (relAccel < fallThreshold) {
        t2 = now; a2 = relAngle;
        st = VALIDATE;
        Serial.printf("  ✦ FALL to VALIDATE (t2=%lu, a2=%.2f)\n", t2, a2);
      }
      break;

    case VALIDATE:
      Serial.printf("  ✦ VALIDATE: dt=%lu, da=%.2f\n", t2 - t1, a2 - a1);
      if ((t2-t1) < MAX_TIME2 && (a2 - a1) > MIN_ANGLE) {
        Serial.printf("    ✔ %s double-click @ %lu\n",
          id==INDEX?"INDEX":"LITTLE", now);
      }
      st = IDLE;
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

float wrap180(float a){
  a = fmod(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}

