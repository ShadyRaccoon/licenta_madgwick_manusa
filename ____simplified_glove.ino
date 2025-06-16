#include "MPU_6050.h"

MPU_6050 mpu(0x68);

long lastPrint = 0;

unsigned long lastAccelSpikeTime = 0;
bool prevAccelAboveThreshold = false;
bool singleClickPending = false;
const long doubleClickTimeout = 2500;

int checkClickSketch(MPU_6050 &mpu) {
  mpu.readAccel();
  float mag = sqrt(mpu.ax_g * mpu.ax_g + mpu.ay_g * mpu.ay_g + mpu.az_g * mpu.az_g);
  Serial.printf("MAG: %.2f | THRESH: %.2f\n", mag, mpu.accelThreshold);
  unsigned long now = millis();
  int clickDetected = 0;

  if (mag > mpu.accelThreshold && !prevAccelAboveThreshold) {
    if (now - lastAccelSpikeTime < doubleClickTimeout) {
      clickDetected = 2;  // double click
      singleClickPending = false;
    } else {
      singleClickPending = true;
    }
    lastAccelSpikeTime = now;
    prevAccelAboveThreshold = true;
  } else if (mag <= mpu.accelThreshold) {
    prevAccelAboveThreshold = false;
  }

  if (singleClickPending && now - lastAccelSpikeTime > doubleClickTimeout) {
    clickDetected = 1;  // single click (optional to ignore)
    singleClickPending = false;
  }

  return clickDetected;
}


void setup() {
  Wire.begin(21, 22,50000);
  Serial.begin(115200);
  selectMuxChannel(2);
  mpu.config();
  Serial.println("CALIBRARE GIROSCOP...");
  delay(1000);
  mpu.computeGyroOffsets(300);
  Serial.println("GIROSCOP CALIBRAT.");
  Serial.println("DETERMINARE POZITIE NEUTRA...");
  delay(1000);
  mpu.computeNeutralAngles(300);
  Serial.println("POZITIE NEUTRA DETERMINATA.");
  Serial.println("DETERMINARE ACCELERATIE MAXIMA...");
  delay(1000);
  mpu.findMaxAccel(5000);
  Serial.println("ACCELERATIE MAXIMA DETERMINATA...");
  Serial.printf("THRESHOLD SET TO: %.2f\n", mpu.accelThreshold);
}

void loop() {
  float roll, pitch;
  selectMuxChannel(2);
  mpu.computeAngles(roll, pitch);
  mpu.readGyro();
  
  if (millis() - lastPrint >= 100) {
    Serial.printf("Roll: %.2f, Pitch: %.2f, Gyro Y: %.2f, Gyro X: %.2f\n", roll, pitch, mpu.gy_dps, mpu.gx_dps);
    Serial.printf("Total: %lu | Faulty: %lu (%.2f%%) | Failed: %lu (%.2f%%)\n",
      mpu.getTotalAccelReadings(),
      mpu.getFaultyAccelReadings(), mpu.getFaultyAccelPercent(),
      mpu.getFailedAccelReadings(), mpu.getFailedAccelPercent());
    lastPrint = millis();
  }
  
  int click = mpu.checkClick();
  //int click = checkClickSketch(mpu);
  //if (click == 1) Serial.println("Single click");
  if (click == 2){
    Serial.printf("Double click AT: %.lu\n", millis());
    delay(3000);
  }
}
