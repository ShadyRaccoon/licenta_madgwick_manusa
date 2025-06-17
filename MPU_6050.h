#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <math.h>
#include <Arduino.h>

void selectMuxChannel(int n);

struct ClickSignature {
  float deltaAngle;
  float peakRelAccel;
  unsigned long timestamp;

  float score() const {
    return deltaAngle * peakRelAccel;
  }
};


class MPU_6050 {
  public:
    uint8_t mpuAddr;
    uint8_t muxChannel;
    const char *name;

    int16_t ax, ay, az, gx, gy, gz;
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float roll, pitch;

    float gyroOffsetX = 0.0f, gyroOffsetY = 0.0f, gyroOffsetZ = 0.0f;
    float neutralRoll = 0.0f, neutralPitch = 0.0f;
    float maxAccelMag = 0.0f;
    float accelThreshold = 0.0f;
    float refAccelThreshold = 0.0f;

    float filteredRoll = 0.0f;
    float filteredPitch = 0.0f;


    MPU_6050(uint8_t address, uint8_t channel, const char *sensorName) : mpuAddr(address), muxChannel(channel), name(sensorName){
      for (int i = 0; i < 30; i++) smoothingBufferAccel[i] = -1;
    }

    void config();
    bool readRaw();
    void readAccel();
    void readGyro();
    void selectMux();
    void computeAngles();
    float getRoll();
    float getPitch();
    void computeGyroOffsets(int samples = 100);
    void computeNeutralAngles(int samples = 100);
    void findMaxAccel(long time = 5000);
    void applyOffsetCorrection();

    void initSensor(int gyroSamples, int neutralSamples, long accelTime, long romTime);

    int getTotalAccelReadings();
    int getFailedAccelReadings();
    int getFaultyAccelReadings();
    float getFailedAccelPercent();
    float getFaultyAccelPercent();

    int checkClick();
    int checkClickRelative(MPU_6050 &ref);
    ClickSignature analyzeClickRelative(MPU_6050 &ref, int windowMs = 200);

    void findRelativeAccelOffset(MPU_6050 &ref, long time);

    void findROM(long time = 5000);
    void setROM(float &minP, float &maxP, float &minR, float &maxR);

    //print functions
    void printROM();
    void printCalibrationData();
    void printDebug();

  private:
    //gesture detection attributes
    unsigned long lastAccelSpikeTime = 0;
    bool prevAccelAboveThreshold = false;
    bool singleClickPending = false;
    const long doubleClickTimeout = 2500;

    //rom attributes
    float minPitch = 91.0f, maxPitch = -91.0f;
    float minRoll  = 01.0f, maxRoll  = -91.0f;

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
