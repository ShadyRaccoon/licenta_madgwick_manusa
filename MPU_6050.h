#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <math.h>

void selectMuxChannel(int n);

class MPU_6050 {
  public:
    uint8_t mpuAddr;

    int16_t ax, ay, az, gx, gy, gz;
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;

    float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
    float neutralRoll = 0, neutralPitch = 0;
    float maxAccelMag = 0;
    float accelThreshold = 0;

    MPU_6050(uint8_t address) : mpuAddr(address) {
      for (int i = 0; i < 30; i++) smoothingBufferAccel[i] = -1;
    }

    void config();
    bool readRaw();
    void readAccel();
    void readGyro();
    void computeAngles(float& roll, float& pitch);
    void computeGyroOffsets(int samples = 100);
    void computeNeutralAngles(int samples = 100);
    void findMaxAccel(int samples = 200);
    void applyOffsetCorrection();

    int getTotalAccelReadings();
    int getFailedAccelReadings();
    int getFaultyAccelReadings();
    float getFailedAccelPercent();
    float getFaultyAccelPercent();

    int checkClick();

  private:
  //gesture detection attributes
  unsigned long lastAccelSpikeTime = 0;
  bool prevAccelAboveThreshold = false;
  bool singleClickPending = false;
  const long doubleClickTimeout = 2500;


  //error tracking attributes
    int totalAccelReadings = 0;
    int faultyAccelReadings = 0;
    int failedAccelReadings = 0;

    uint8_t crtIndexBuffer = 0;
    float smoothingBufferAccel[30];
    bool bufferedOnceAccel = false;
  //error tracking methods
    void smoothAccel(float *ax_g, float *ay_g, float *az_g, bool isValid);
    void smoothGyro(float *gx_dps, float *gy_dps, float *gz_dps);

  //gesture detectrion methods
};

#endif
