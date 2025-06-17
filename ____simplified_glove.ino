#include "MPU_6050.h"

MPU_6050 mpuINDX(0x68, 1, "INDX");
MPU_6050 mpuLITT(0x68, 2, "LITT");
MPU_6050 mpuPALM(0x68, 0, "PALM");

long lastPrint = 0;

void setup() {
  Wire.begin(21, 22,50000);
  Serial.begin(115200);
  mpuPALM.initSensor(300,300,0,5000);
  mpuPALM.printCalibrationData();

  mpuINDX.initSensor(0,0,5000,0);
  mpuINDX.printCalibrationData();  

  mpuLITT.initSensor(0,0,5000,0);
  mpuLITT.printCalibrationData();

  Serial.println("CALCULARE PRAG DIFERENTA MAGNITUDINE ACCELERATIE INDX & LITT FATA DE PALM");
  Serial.println("INDX...");
  delay(1000);

  mpuINDX.findRelativeAccelOffset(mpuPALM, 2000);
  Serial.printf("INDEX PRAG OFFSET: %.3f\n", mpuINDX.refAccelThreshold);
  Serial.println("LITT...");
  delay(1000);

  mpuLITT.findRelativeAccelOffset(mpuPALM, 2000);
  Serial.printf("LITT PRAG OFFSET: %.3f\n", mpuLITT.refAccelThreshold);
  delay(1000);
}

void loop() {
  mpuINDX.computeAngles();
  float roll = mpuINDX.getRoll();
  float pitch = mpuINDX.getPitch();
  mpuINDX.readGyro();
  
  //if (millis() - lastPrint >= 100) {
  //  mpuINDX.printDebug();
  //  lastPrint = millis();
  //}
  
  if (mpuINDX.checkClickRelative(mpuPALM) == 2 || mpuLITT.checkClickRelative(mpuPALM) == 2) {
    Serial.println("Double click detected — analyzing...");

    ClickSignature indexSig = mpuINDX.analyzeClickRelative(mpuPALM, 500);
    ClickSignature littleSig = mpuLITT.analyzeClickRelative(mpuPALM, 500);

    float littleScore = littleSig.score();
    float indexScore = indexSig.score();

    Serial.printf("INDEX SCORE:  %.2f (ΔAngle: %.2f, RelAccel: %.2f)\n",
      indexScore, indexSig.deltaAngle, indexSig.peakRelAccel);
    Serial.printf("LITTLE SCORE: %.2f (ΔAngle: %.2f, RelAccel: %.2f)\n",
      littleScore, littleSig.deltaAngle, littleSig.peakRelAccel);

    if (indexScore > 0.3f && littleScore > 0.3f){
      if (indexScore > littleScore) {
        Serial.printf("✔ INDEX finger click @ %lu\n", indexSig.timestamp);
      } else {
        Serial.printf("✔ LITTLE finger click @ %lu\n", littleSig.timestamp);
      }
      delay(3000);
    }
  }
}
