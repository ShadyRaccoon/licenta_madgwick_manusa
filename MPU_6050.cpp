#define PI 3.14159265358979323846
#include "MPU_6050.h"

void selectMuxChannel(int channel) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void MPU_6050::config() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();  // Wake up
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();  // Gyro config
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();  // LPF
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();  // Accel config
}

bool MPU_6050::readRaw() {
  totalAccelReadings++;
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(mpuAddr, 14, true) != 14) {
    failedAccelReadings++;
    return false;
  }

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
  return true;
}

void MPU_6050::readAccel() {
  if (!readRaw()) {
    totalAccelReadings++;
    smoothAccel(&ax_g, &ay_g, &az_g, false);
    return;
  }
  ax_g = ax / 4096.0f;
  ay_g = ay / 4096.0f;
  az_g = az / 4096.0f;

  float mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  if(mag < 4.0f) mag = 4.0f;
  bool isValid = (mag > 0.1f);
  if (!isValid) faultyAccelReadings++;
  smoothAccel(&ax_g, &ay_g, &az_g, isValid);
}

void MPU_6050::smoothAccel(float *ax_g, float *ay_g, float *az_g, bool isValid) {
  float mag = sqrt((*ax_g) * (*ax_g) + (*ay_g) * (*ay_g) + (*az_g) * (*az_g));
  smoothingBufferAccel[crtIndexBuffer] = isValid ? mag : -1;
  crtIndexBuffer = (crtIndexBuffer + 1) % 30;

  if (!bufferedOnceAccel && crtIndexBuffer == 0) bufferedOnceAccel = true;

  float sum = 0.0f;
  int samples = bufferedOnceAccel ? 30 : crtIndexBuffer;
  int validSamples = 0;
  for (int i = 0; i < samples; i++) {
    if (smoothingBufferAccel[i] != -1) {
      sum += smoothingBufferAccel[i];
      validSamples++;
    }
  }

  float avgMag = validSamples > 0 ? sum / validSamples : mag;
  float currentMag = mag;
  if (currentMag > 0.001) {
    float scale = avgMag / currentMag;
    *ax_g *= scale;
    *ay_g *= scale;
    *az_g *= scale;
  }
}

void MPU_6050::readGyro() {
  readRaw();
  gx_dps = gx / 65.5f - gyroOffsetX;
  gy_dps = gy / 65.5f - gyroOffsetY;
  gz_dps = gz / 65.5f - gyroOffsetZ;
}

void MPU_6050::computeAngles(float& roll, float& pitch) {
  readAccel();
  roll = atan2(ay_g, az_g) * 180.0f / PI;
  pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0f / PI;
}

void MPU_6050::computeGyroOffsets(int samples) {
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    readRaw();
    sumX += gx; sumY += gy; sumZ += gz;
    delay(5);
  }
  gyroOffsetX = (sumX / samples) / 65.5f;
  gyroOffsetY = (sumY / samples) / 65.5f;
  gyroOffsetZ = (sumZ / samples) / 65.5f;
}

void MPU_6050::computeNeutralAngles(int samples) {
  float rollSum = 0, pitchSum = 0, r, p;
  for (int i = 0; i < samples; i++) {
    computeAngles(r, p);
    rollSum += r;
    pitchSum += p;
    delay(5);
  }
  neutralRoll = rollSum / samples;
  neutralPitch = pitchSum / samples;
}

void MPU_6050::findMaxAccel(int time) {
  maxAccelMag = 0;
  long start = millis();
  while(millis() - start < time){
    readAccel();
    float mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    if (mag > maxAccelMag) maxAccelMag = mag;
    delay(5);
  }
  accelThreshold = maxAccelMag * 0.85f;
}

void MPU_6050::applyOffsetCorrection() {
  gx_dps -= gyroOffsetX;
  gy_dps -= gyroOffsetY;
  gz_dps -= gyroOffsetZ;
}

int MPU_6050::getTotalAccelReadings() { return totalAccelReadings; }
int MPU_6050::getFailedAccelReadings() { return failedAccelReadings; }
int MPU_6050::getFaultyAccelReadings() { return faultyAccelReadings; }

float MPU_6050::getFailedAccelPercent() {
  return totalAccelReadings > 0 ? (100.0f * failedAccelReadings) / totalAccelReadings : 0.0f;
}

float MPU_6050::getFaultyAccelPercent() {
  return totalAccelReadings > 0 ? (100.0f * faultyAccelReadings) / totalAccelReadings : 0.0f;
}

int MPU_6050::checkClick() {
  readAccel();
  float mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  unsigned long now = millis();
  int clickDetected = 0;

  if (mag > accelThreshold && !prevAccelAboveThreshold) {
    if (now - lastAccelSpikeTime < doubleClickTimeout) {
      clickDetected = 2;  // double click
      singleClickPending = false;
    } else {
      singleClickPending = true;
    }
    lastAccelSpikeTime = now;
    prevAccelAboveThreshold = true;
  } else if (mag <= accelThreshold) {
    prevAccelAboveThreshold = false;
  }

  if (singleClickPending && now - lastAccelSpikeTime > doubleClickTimeout) {
    clickDetected = 1;  // single click (optional to ignore)
    singleClickPending = false;
  }

  return clickDetected;
}

