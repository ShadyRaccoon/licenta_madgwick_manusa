  #ifndef MPU6050_H
  #define MPU6050_H

  #include <Wire.h>
  #include <math.h>
  #include <Arduino.h>
  #include "MadgwickAHRS.h"

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
      float neutralAX = 0.0f, neutralAY = 0.0f;
      float accelThreshold = 0.0f;

      float filteredRoll = 0.0f;
      float filteredPitch = 0.0f;
      
      Madgwick filter;       // one per sensor
      float q0=1, q1=0, q2=0, q3=0;  // quaternion components

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
      float getMinRoll();
      float getMaxRoll();
      float getMinPitch();
      float getMaxPitch();
      float getNeutralRoll();
      float getNeutralPitch();
      float getNeutralAX();
      float getNeutralAY();
      float getNeutralAZ();
      void computeGyroOffsets(int samples = 100);
      void applyOffsetCorrection();
      void computeNeutralAngles(int samples = 100);
      void computeNeutralAccelerations(int samples = 100);

      void initSensor(int gyroSamples, int neutralSamples, long romTime);

      int getTotalAccelReadings();
      int getFailedAccelReadings();
      int getFaultyAccelReadings();
      float getFailedAccelPercent();
      float getFaultyAccelPercent();
      float getMaxAX();
      float getMinAX();
      float getMaxAY();
      float getMinAY();

      void findRelativeAccelOffset(MPU_6050 &ref, long time);

      void findROM(long time = 5000);
      void setROM(float &minP, float &maxP, float &minR, float &maxR);

      //print functions
      void printROM();
      void printCalibrationData();
      void printDebug();

    private:
      //gesture detection attributes  
      bool prevAccelAboveThreshold = false;

      //rom attributes
      float minPitch = 91.0f, maxPitch = -91.0f;
      float minRoll  = 01.0f, maxRoll  = -91.0f;

      float maxAX = -1.0f, minAX = 1.0f;
      float maxAY = -1.0f, minAY = 1.0f;
      float maxAZ = -1.0f, minAZ = 1.0f;

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
