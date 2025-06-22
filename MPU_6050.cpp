#define PI 3.14159265358979323846
#include "MPU_6050.h"

void selectMuxChannel(int channel) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(10);
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
  selectMux();
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

void MPU_6050::findROM(long time){
  long start = millis();
  while(millis() - start < time){
    readAccel();
    if(ax_g > maxAX) maxAX = ax_g;
    if(ax_g < minAX) minAX = ax_g;
    if(ay_g > maxAY) maxAY = ay_g;
    if(ay_g < minAY) minAY = ay_g; 
    //delay(5);
  }
}

void MPU_6050::setROM(float &minP, float &maxP, float &minR, float &maxR){
  maxRoll = maxR;
  minRoll = minR;
  maxPitch = maxP;
  minPitch = minP;
}

void MPU_6050::printROM(){
  Serial.printf("[%s] -> MAX_ROLL: %.2f | MIN_ROLL: %.2f | MAX_PITCH: %.2f | MIN_PITCH: %.2f\n", name, maxRoll, minRoll, maxPitch, minPitch);
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

void MPU_6050::computeAngles() {
  readAccel();
  float rawRoll = atan2(ay_g, az_g) * 180.0f / PI;
  float rawPitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0f / PI;

  const float alpha = 0.7f;  // Smoothing factor (0.8–0.95)

  filteredRoll  = alpha * filteredRoll  + (1 - alpha) * rawRoll;
  filteredPitch = alpha * filteredPitch + (1 - alpha) * rawPitch;

  roll = filteredRoll;
  pitch = filteredPitch;
}

void MPU_6050::computeGyroOffsets(int samples) {
  if(samples == 0) return;
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    readRaw();
    sumX += gx; sumY += gy; sumZ += gz;
    //delay(10);
  }
  gyroOffsetX = (sumX / samples) / 65.5f;
  gyroOffsetY = (sumY / samples) / 65.5f;
  gyroOffsetZ = (sumZ / samples) / 65.5f;
}

void MPU_6050::computeNeutralAngles(int samples) {
  float rollSum = 0, pitchSum = 0;
  if(samples == 0) return;
  for (int i = 0; i < samples; i++) {
    computeAngles();
    rollSum += roll;
    pitchSum += pitch;
    //delay(5);
  }
  neutralRoll = rollSum / samples;
  neutralPitch = pitchSum / samples;
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

void MPU_6050::printDebug() {
  Serial.printf("Roll: %.2f, Pitch: %.2f, Gyro Y: %.2f, Gyro X: %.2f\n", 
                roll, pitch,gy_dps, gx_dps);
  Serial.printf("Total: %lu | Faulty: %lu (%.2f%%) | Failed: %lu (%.2f%%)\n",
                getTotalAccelReadings(),
                getFaultyAccelReadings(), getFaultyAccelPercent(),
                getFailedAccelReadings(), getFailedAccelPercent());
}

void MPU_6050::initSensor(int gyroSamples, int neutralSamples, long romTime){
  selectMux();
  config();
  Serial.printf("[%s] CALIBRARE GIROSCOP...\n", name);
  delay(  1000);
  computeGyroOffsets(gyroSamples);
  Serial.printf("[%s] GIROSCOP CALIBRAT.\n", name);
  Serial.printf("[%s] DETERMINARE POZITIE NEUTRA...\n", name);
  delay(1000);
  computeNeutralAccelerations(neutralSamples);
  Serial.printf("[%s] POZITIE NEUTRA DETERMINATA.\n", name);
  Serial.printf("[%s] DETERMINARE RANGE OF MOTION...\n", name);
  delay(1000);
  findROM(romTime);
  Serial.printf("[%s] RANGE OF MOTION DETERMINAT...\n", name);
}

void MPU_6050::printCalibrationData() {
  Serial.printf("[%s] OFFSET GX: %.2f, GY: %.2f, GZ: %.2f\n", name, gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("[%s] NEUTRAL ROLL: %.2f, PITCH: %.2f\n", name, neutralRoll, neutralPitch);
  Serial.printf("[%s] THRESHOLD: %.2f\n", name, accelThreshold);
  Serial.printf("[%s] ROM - ROLL: (%.2f to %.2f), PITCH: (%.2f to %.2f)\n", name,
                minRoll, maxRoll, minPitch, maxPitch);
}

float MPU_6050::getRoll(){
  return roll;
}

float MPU_6050::getPitch(){
  return pitch;
}

void MPU_6050::selectMux(){
  ::selectMuxChannel(muxChannel);
}

void MPU_6050::computeNeutralAccelerations(int samples){
  float axSum = 0, aySum = 0;
  if(samples == 0) return;
  for (int i = 0; i < samples; i++) {
    readAccel();
    axSum += ax_g;
    aySum += ay_g;
  }
  neutralAX = axSum / samples;
  neutralAY = aySum / samples;
}

float MPU_6050::getMinRoll(){ return minRoll; }
float MPU_6050::getMaxRoll(){ return maxRoll; }
float MPU_6050::getMinPitch(){ return minPitch; }
float MPU_6050::getMaxPitch(){ return maxPitch; }

/*
    float minAY = 1.0f, maxAY = -1.0f;
    float minAX = 1.0f, maxAX = -1.0f;

    float neutralAX = 0.0f, neutralAY = 0.0f;
*/
float MPU_6050::getMaxAX() { return maxAX; }
float MPU_6050::getMinAX() { return minAX; }
float MPU_6050::getMaxAY() { return maxAY; }
float MPU_6050::getMinAY() { return minAY; }
float MPU_6050::getNeutralAX() { return neutralAX; }
float MPU_6050::getNeutralAY() { return neutralAY; }