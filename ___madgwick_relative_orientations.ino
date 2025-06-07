#include <Wire.h>
#include <Preferences.h>
#include <MadgwickAHRS.h>

#define RED 5
#define GREEN 18
#define BLUE 23

#define THUMB 4
#define INDEX 2
#define LITTLE 15

#define FLEX_REFERENCE_TIME 5000

#define CAL_SAMPLES 150

#define FA 0
#define UA 2
#define PALM 7

#define INTERVAL 10
#define SAMPLE_FREQ 100

Preferences prefs;

int thumb_min = 4095, thumb_max = 0;
int index_min = 4095, index_max = 0;
int little_min = 4095, little_max = 0;

long last_print = 0;

float gX_palm, gY_palm, gZ_palm;
float gX_fa, gY_fa, gZ_fa;
float gX_ua, gY_ua, gZ_ua;

float offX_palm, offY_palm, offZ_palm;
float offX_fa, offY_fa, offZ_fa;
float offX_ua, offY_ua, offZ_ua;

float aX_palm, aY_palm, aZ_palm;
float aX_fa, aY_fa, aZ_fa;
float aX_ua, aY_ua, aZ_ua;

float q0_palm, q1_palm, q2_palm, q3_palm;
float q0_fa, q1_fa, q2_fa, q3_fa;
float q0_ua, q1_ua, q2_ua, q3_ua;

float roll_palm, pitch_palm, yaw_palm;
float roll_fa, pitch_fa, yaw_fa;
float roll_ua, pitch_ua, yaw_ua;

Madgwick filter_palm, filter_fa, filter_ua;

void mpu_config(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00); //sleep bit -> 0
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //config gyro range
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //turn on LPF
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68); //config accel range
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}

void mux_channel(uint8_t channel){
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void gyro_data(float *gX, float *gY, float *gZ){
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68,6);

  int16_t rawX = Wire.read() << 8 | Wire.read();
  int16_t rawY = Wire.read() << 8 | Wire.read();
  int16_t rawZ = Wire.read() << 8 | Wire.read();

  //convert raw value to deg/s
  *gX = (float)rawX / 65.5;
  *gY = (float)rawY / 65.5;
  *gZ = (float)rawZ / 65.5;
}

void print_gyro(char * segment, float gX, float gY, float gZ){
  Serial.print("Viteza unghiulara pe X pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.println(gX);

  Serial.print("Viteza unghiulara pe Y pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.println(gY);

  Serial.print("Viteza unghiulara pe Z pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.println(gZ);
}

void get_gyro_offsets(float *offX, float *offY, float *offZ){
  float x = 0, y = 0, z = 0;
  float gX, gY, gZ;
  for(int i = 0 ; i < CAL_SAMPLES ; i++){
    gyro_data(&gX, &gY, &gZ);
    x += gX;
    y += gY;
    z += gZ;
  }

  *offX = (float) x / CAL_SAMPLES;
  *offY = (float) y / CAL_SAMPLES;
  *offZ = (float) z / CAL_SAMPLES; 
}

void calibrate_gyro(float *gX, float *gY, float *gZ, float offX, float offY, float offZ){
  *gX -= offX;
  *gY -= offY;
  *gZ -= offZ;
}

void accel_data(float *aX, float *aY, float *aZ){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(0x68,6);

  int16_t rawX = Wire.read() << 8 | Wire.read();
  int16_t rawY = Wire.read() << 8 | Wire.read();
  int16_t rawZ = Wire.read() << 8 | Wire.read();

  //convert raw value to deg/s
  *aX = (float)rawX / 4096;
  *aY = (float)rawY / 4096;
  *aZ = (float)rawZ / 4096;
}

void print_accel(char *segment, float aX, float aY, float aZ){
  Serial.print("Acceleratia pe X pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.println(aX);

  Serial.print("Acceleratia pe Y pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.println(aY);

  Serial.print("Acceleratia pe Z pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.println(aZ);
}

void print_angles_quaternions(char *segment, float q0, float q1, float q2, float q3, float roll, float pitch, float yaw){
  Serial.printf(" Quaternion (%s): [%.3f  %.3f  %.3f  %.3f]\n", segment, q0, q1, q2, q3);

  Serial.printf(" Euler R/P/Y (%s): [%.1f°,  %.1f°,  %.1f°]\n\n", segment, roll, pitch, yaw);
}

void get_flex_values(){
  int thumb_val = analogRead(THUMB);
  int index_val = analogRead(INDEX);
  int little_val = analogRead(LITTLE);

  thumb_min = min(thumb_min, thumb_val);
  thumb_max = max(thumb_max, thumb_val);

  index_min = min(index_min, index_val);
  index_max = max(index_max, index_val);

  little_min = min(little_min, little_val);
  little_max = max(little_max, little_val);
}

void save_flex_range(){
  prefs.begin("flexRange", false);

  prefs.putInt("thumb_min", thumb_min);
  prefs.putInt("thumb_max", thumb_max);

  prefs.putInt("index_min", index_min);
  prefs.putInt("index_max", index_max);

  prefs.putInt("little_min", little_min);
  prefs.putInt("little_max", little_max);

  prefs.end();

  Serial.println("RoM of flex sensors saved.");
}

void load_flex_range(){
  prefs.begin("flexRange", true);

  thumb_min = prefs.getInt("thumb_min", 4095);
  thumb_max = prefs.getInt("thumb_max", 0);

  index_min = prefs.getInt("index_min", 4095);
  index_max = prefs.getInt("index_max", 0);

  little_min = prefs.getInt("little_min", 4095);
  little_max = prefs.getInt("little_max", 0);

  prefs.end();

  Serial.println("RoM of flex sensors loaded.");
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  prefs.begin("gyroCal", false);
  bool isCalibrated = prefs.getBool("calibrated", false);
  bool flexReference = prefs.getBool("flexRange", false);

  if(!isCalibrated){
    Serial.println("Senzori necalibrati...");
    Serial.println("Calibrare...");

    Serial.println("Lasati senzorii nemiscati timp de 5s");
    delay(5000);

    mux_channel(PALM);
    delay(10);
    mpu_config();
    delay(10);
    get_gyro_offsets(&offX_palm, &offY_palm, &offZ_palm);

    Serial.println("== Calibrare senzor antebrat ==");
    mux_channel(FA);
    delay(10);
    mpu_config();
    delay(10);
    get_gyro_offsets(&offX_fa, &offY_fa, &offZ_fa);

    Serial.println("== Calibrare senzor brat ==");
    mux_channel(UA);
    delay(10);
    mpu_config();
    delay(10);
    get_gyro_offsets(&offX_ua, &offY_ua, &offZ_ua);

    Serial.println("== Calibrare completa. Salvare in flash... ==");

    prefs.putFloat("pX", offX_palm);
    prefs.putFloat("pY", offY_palm);
    prefs.putFloat("pZ", offZ_palm);

    prefs.putFloat("fX", offX_fa);
    prefs.putFloat("fY", offY_fa);
    prefs.putFloat("fZ", offZ_fa);

    prefs.putFloat("uX", offX_ua);
    prefs.putFloat("uY", offY_ua);
    prefs.putFloat("uZ", offZ_ua);

    prefs.putBool("calibrated", true);
    prefs.end();

    Serial.println(">>> Date de calibrare salvate. Reporniti alimentarea <<<");
    while(true){}
  }

  if(!flexReference){
    Serial.println("Senzori de rezistenta necalibrati...");
    Serial.println("Calibrare...");

    int start = millis();

    

    while(millis() - start < FLEX_REFERENCE_TIME){
      get_flex_values();
    }

    save_flex_range();

    Serial.println(">>> RoM senzori de rezistenta salvat. Reporniti alimentarea <<<");

    Serial.println("=== Flex Sensors Range of Motion ===");

    Serial.print("Thumb Sensor RoM: Min = ");
    Serial.print(thumb_min);
    Serial.print(", Max = ");
    Serial.println(thumb_max);

    Serial.print("Index Sensor RoM: Min = ");
    Serial.print(index_min);
    Serial.print(", Max = ");
    Serial.println(index_max);

    Serial.print("Little Sensor RoM: Min = ");
    Serial.print(little_min);
    Serial.print(", Max = ");
    Serial.println(little_max);

    Serial.println("=====================================");

    while(true){};
  }

  mux_channel(PALM);
  delay(10);
  mpu_config();
  delay(10);

  mux_channel(FA);
  delay(10);
  mpu_config();
  delay(10);

  mux_channel(UA);
  delay(10);
  mpu_config();
  delay(10);

  Serial.println("== Calibrare... ==");

  offX_palm = prefs.getFloat("pX", 0.0f);
  offY_palm = prefs.getFloat("pY", 0.0f);
  offZ_palm = prefs.getFloat("pZ", 0.0f);

  offX_fa   = prefs.getFloat("fX", 0.0f);
  offY_fa   = prefs.getFloat("fY", 0.0f);
  offZ_fa   = prefs.getFloat("fZ", 0.0f);

  offX_ua   = prefs.getFloat("uX", 0.0f);
  offY_ua   = prefs.getFloat("uY", 0.0f);
  offZ_ua   = prefs.getFloat("uZ", 0.0f);

  prefs.end();

  Serial.println(">>> Senzori calibrati <<<");

  filter_palm.begin(SAMPLE_FREQ);
  filter_fa.begin(SAMPLE_FREQ);
  filter_ua.begin(SAMPLE_FREQ);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  mux_channel(PALM);
  delay(5);
  gyro_data(&gX_palm, &gY_palm, &gZ_palm);
  accel_data(&aX_palm, &aY_palm, &aZ_palm);
  calibrate_gyro(&gX_palm, &gY_palm, &gZ_palm, offX_palm, offY_palm, offZ_palm);
  filter_palm.updateIMU(gX_palm, gY_palm, gZ_palm, aX_palm, aY_palm, aZ_palm);
  q0_palm = filter_palm.getQuaternion0();
  q1_palm = filter_palm.getQuaternion1();
  q2_palm = filter_palm.getQuaternion2();
  q3_palm = filter_palm.getQuaternion3();
  roll_palm  = filter_palm.getRoll();
  pitch_palm = filter_palm.getPitch();
  yaw_palm   = filter_palm.getYaw();

  mux_channel(FA);
  delay(5);
  gyro_data(&gX_fa, &gY_fa, &gZ_fa);
  accel_data(&aX_fa, &aY_fa, &aZ_fa);
  calibrate_gyro(&gX_fa, &gY_fa, &gZ_fa, offX_fa, offY_fa, offZ_fa);
  filter_fa.updateIMU(gX_fa, gY_fa, gZ_fa, aX_fa, aY_fa, aZ_fa);
  q0_fa = filter_fa.getQuaternion0();
  q1_fa = filter_fa.getQuaternion1();
  q2_fa = filter_fa.getQuaternion2();
  q3_fa = filter_fa.getQuaternion3();
  roll_fa  = filter_fa.getRoll();
  pitch_fa = filter_fa.getPitch();
  yaw_fa   = filter_fa.getYaw();

  mux_channel(UA);
  delay(5);
  gyro_data(&gX_ua, &gY_ua, &gZ_ua);
  accel_data(&aX_ua, &aY_ua, &aZ_ua);
  calibrate_gyro(&gX_ua, &gY_ua, &gZ_ua, offX_ua, offY_ua, offZ_ua);
  filter_ua.updateIMU(gX_ua, gY_ua, gZ_ua, aX_ua, aY_ua, aZ_ua);
  q0_ua = filter_ua.getQuaternion0();
  q1_ua = filter_ua.getQuaternion1();
  q2_ua = filter_ua.getQuaternion2();
  q3_ua = filter_ua.getQuaternion3();
  roll_ua  = filter_ua.getRoll();
  pitch_ua = filter_ua.getPitch();
  yaw_ua   = filter_ua.getYaw();

  if( millis() - last_print >= 1000){
    print_gyro("palma", gX_palm, gY_palm, gZ_palm);
    print_accel("palma", aX_palm, aY_palm, aZ_palm);
    print_angles_quaternions("palma ", q0_palm, q1_palm, q2_palm, q3_palm, rofll_palm, pitch_palm, yaw_palm);
    
    print_gyro("antebrat", gX_fa, gY_fa, gZ_fa);
    print_accel("antebrat", aX_fa, aY_fa, aZ_fa);
    print_angles_quaternions("antebrat ", q0_fa, q1_fa, q2_fa, q3_fa, roll_fa, pitch_fa, yaw_fa);

    print_gyro("brat", gX_ua, gY_ua, gZ_ua);
    print_accel("brat", aX_ua, aY_ua, aZ_ua);
    print_angles_quaternions("brat ", q0_ua, q1_ua, q2_ua, q3_ua, roll_ua, pitch_ua, yaw_ua);

    last_print = millis();  
  }
  //delay(500);
}
