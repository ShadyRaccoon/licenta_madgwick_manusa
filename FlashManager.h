#ifndef FLASH_MANAGER_H
#define FLASH_MANAGER_H

#include <Preferences.h>
#include "MPU_6050.h"
#include <Arduino.h>

class FlashManager {
  public:
    FlashManager() {}

    bool calibrationExists(const String& sensorName) {
      String key = "mpu_" + sensorName;
      prefs.begin(key.c_str(), true);
      bool exists = prefs.isKey("thresh");
      prefs.end();
      return exists;
    }

    void saveCalibration(const MPU_6050& mpu, const String& sensorName) {
      String key = "mpu_" + sensorName;
      prefs.begin(key.c_str(), false);
      prefs.putFloat("thresh", mpu.accelThreshold);
      prefs.putFloat("nRoll",  mpu.neutralRoll);
      prefs.putFloat("nPitch", mpu.neutralPitch);
      prefs.putFloat("gOffX",  mpu.gyroOffsetX);
      prefs.putFloat("gOffY",  mpu.gyroOffsetY);
      prefs.putFloat("gOffZ",  mpu.gyroOffsetZ);
      prefs.end();
    }


    void loadCalibration(MPU_6050& mpu, const String& sensorName) {
      String key = "mpu_" + sensorName;
      prefs.begin(key.c_str(), true);
      mpu.setAccelThreshold(prefs.getFloat("thresh", 1.0f));
      mpu.setNeutralAngles(prefs.getFloat("nRoll", 0.0f), prefs.getFloat("nPitch", 0.0f));
      mpu.setGyroOffsets(
        prefs.getFloat("gOffX", 0.0f),
        prefs.getFloat("gOffY", 0.0f),
        prefs.getFloat("gOffZ", 0.0f)
      );
      prefs.end();
    }

    void loadOrCalibrate(MPU_6050& mpu, const String& sensorName, int gyroSamples, int angleSamples, int accelSamples, bool forceRecalibrate = false) {
      if (!calibrationExists(sensorName) || forceRecalibrate) {
        Serial.printf("CALIBRARE SENZOR '%s'...\n", sensorName.c_str());
        Serial.println("TINETI MANA NEMISCATA. CALIBRARE GIROSCOP...");
        mpu.computeGyroOffsets(gyroSamples);
        Serial.println("GIROSCOP CALIBRAT...");
        Serial.println("TINETI MANA NEMISCATA, PALMA PARALELA CU PODEAUA...");
        delay(1000);
        mpu.computeNeutralAngles(angleSamples);
        Serial.println("POZITIE NEUTRA DETERMINATA...");
        Serial.println("FLEXATI DEGETUL. DETERMINARE ACCELERATIE MAXIMA...");
        delay(1000);
        mpu.findMaxAccel(accelSamples);
        Serial.println("ACCELERATIE MAXIMA DETERMINATA...");
        Serial.println("SALVARE DATE CALIBRARE...");
        delay(1000);
        saveCalibration(mpu, sensorName);
        Serial.printf("CALIBRARE COMPLETA PENTRU '%s'\n", sensorName.c_str());
      } else {
        Serial.printf("INCARCARE DATE CALIBRARE '%s'...\n", sensorName.c_str());
        loadCalibration(mpu, sensorName);
      }
    }

  private:
    Preferences prefs;
};

#endif
