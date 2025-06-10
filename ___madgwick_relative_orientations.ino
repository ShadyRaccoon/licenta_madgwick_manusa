#include <Wire.h>
#include <Preferences.h>
#include <MadgwickAHRS.h>

#define RED 5
#define GREEN 18
#define BLUE 23

#define THUMB 4
#define INDEX 2
#define LITTLE 15

#define WAIT 3000
#define FLEX_REFERENCE_TIME 10000
#define NEUTRAL_REFERENCE_TIME 10000
#define PALM_ROM_REFERENCE_TIME 10000

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

float pitch_min = 10000, pitch_max = -10000;
float pQ0_pitch_min, pQ1_pitch_min, pQ2_pitch_min, pQ3_pitch_min; 
float pQ0_pitch_max, pQ1_pitch_max, pQ2_pitch_max, pQ3_pitch_max; 

float roll_min = 10000, roll_max = -10000;
float pQ0_roll_min, pQ1_roll_min, pQ2_roll_min, pQ3_roll_min; 
float pQ0_roll_max, pQ1_roll_max, pQ2_roll_max, pQ3_roll_max; 

float q0_palm_neutral, q1_palm_neutral, q2_palm_neutral, q3_palm_neutral;
float q0_fa_neutral, q1_fa_neutral, q2_fa_neutral, q3_fa_neutral;
float q0_ua_neutral, q1_ua_neutral, q2_ua_neutral, q3_ua_neutral;

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

void led_color(int red, int green, int blue){
  analogWrite(RED, red);
  analogWrite(GREEN, green);
  analogWrite(BLUE, blue);
}

void led_wait(){
  unsigned long start = millis();
  unsigned long lastToggle = start;
  bool on = true;

  while (millis() - start < WAIT) {
    if (millis() - lastToggle >= 300) {
      on = !on;
      lastToggle = millis();
      if (on) {
        led_color(255, 165, 0);
      } else {
        led_color(0, 0, 0);
      }
    }
  }
}

void led_hold(){
  led_color(255, 0, 255);
}

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

void config_all() {
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
  Serial.print("== Viteza unghiulara pe X pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.print(gX);
  Serial.println(" ==");

  Serial.print("== Viteza unghiulara pe Y pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.print(gY);
  Serial.println(" ==");

  Serial.print("== Viteza unghiulara pe Z pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.print(gZ);
  Serial.println(" ==");
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
  Serial.print("== Acceleratia pe X pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.print(aX);
  Serial.println(" ==");

  Serial.print("== Acceleratia pe Y pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.print(aY);
  Serial.println(" ==");

  Serial.print("== Acceleratia pe Z pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.print(aZ);
  Serial.println(" ==");
}

void print_angles_quaternions(char *segment, float q0, float q1, float q2, float q3, float roll, float pitch, float yaw){
  Serial.printf("== Quaternion (%s): [%.3f  %.3f  %.3f  %.3f] ==\n", segment, q0, q1, q2, q3);

  Serial.printf("== Euler R/P/Y (%s): [%.1f°,  %.1f°,  %.1f°] ==\n\n", segment, roll, pitch, yaw);
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

  prefs.putBool("flexRoM", true);
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

void load_gyro_offsets() {
  prefs.begin("gyroCal", true);

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
}

void save_neutral_pose(){
  prefs.begin("neutralPose", false);
  prefs.putFloat("pW", q0_palm_neutral);
  prefs.putFloat("pX", q1_palm_neutral);
  prefs.putFloat("pY", q2_palm_neutral);
  prefs.putFloat("pZ", q3_palm_neutral);
  prefs.putFloat("fW", q0_fa_neutral);
  prefs.putFloat("fX", q1_fa_neutral);
  prefs.putFloat("fY", q2_fa_neutral);
  prefs.putFloat("fZ", q3_fa_neutral);
  prefs.putFloat("uW", q0_ua_neutral);
  prefs.putFloat("uX", q1_ua_neutral);
  prefs.putFloat("uY", q2_ua_neutral);
  prefs.putFloat("uZ", q3_ua_neutral);
  prefs.putBool("neutral_pose", true);
  prefs.end();
}

void load_neutral_pose(){
  prefs.begin("neutralPose", true);
  q0_palm_neutral = prefs.getFloat("pW", 1.0f);
  q1_palm_neutral = prefs.getFloat("pX", 0.0f);
  q2_palm_neutral = prefs.getFloat("pY", 0.0f);
  q3_palm_neutral = prefs.getFloat("pZ", 0.0f);
  q0_fa_neutral = prefs.getFloat("fW", 1.0f);
  q1_fa_neutral = prefs.getFloat("fX", 0.0f);
  q2_fa_neutral = prefs.getFloat("fY", 0.0f);
  q3_fa_neutral = prefs.getFloat("fZ", 0.0f);
  q0_ua_neutral = prefs.getFloat("uW", 1.0f);
  q1_ua_neutral = prefs.getFloat("uX", 0.0f);
  q2_ua_neutral = prefs.getFloat("uY", 0.0f);
  q3_ua_neutral = prefs.getFloat("uZ", 0.0f);
  prefs.end();

  Serial.println("Neutral reference loaded.");
}

void print_neutral_pose() {
  Serial.println(F("=== Neutral Pose Reference ==="));
  
  Serial.printf("== Palm Neutral Q: [%.3f, %.3f, %.3f, %.3f] ==\n",
                q0_palm_neutral,
                q1_palm_neutral,
                q2_palm_neutral,
                q3_palm_neutral);

  Serial.printf("== Forearm (FA) Neutral Q: [%.3f, %.3f, %.3f, %.3f] ==\n",
                q0_fa_neutral,
                q1_fa_neutral,
                q2_fa_neutral,
                q3_fa_neutral);

  Serial.printf("== Upper-arm (UA) Neutral Q: [%.3f, %.3f, %.3f, %.3f] ==\n",
                q0_ua_neutral,
                q1_ua_neutral,
                q2_ua_neutral,
                q3_ua_neutral);
}

void capture_neutral_pose(){
    Serial.println("Obtinere date de referinta pt pozitie neutra");

    led_wait();

    int start = millis();
    int samples = 0;
    float sumPw = 0, sumPx = 0, sumPy = 0, sumPz = 0;
    float sumFw = 0, sumFx = 0, sumFy = 0, sumFz = 0;
    float sumUw = 0, sumUx = 0, sumUy = 0, sumUz = 0;

    led_hold();

    while(millis() - start < NEUTRAL_REFERENCE_TIME){
      mux_channel(PALM);
      gyro_data(&gX_palm, &gY_palm, &gZ_palm);
      accel_data(&aX_palm, &aY_palm, &aZ_palm);
      filter_palm.updateIMU(gX_palm, gY_palm, gZ_palm, aX_palm, aY_palm, aZ_palm);
      sumPw += filter_palm.getQuaternion0();
      sumPx += filter_palm.getQuaternion1();
      sumPy += filter_palm.getQuaternion2();
      sumPz += filter_palm.getQuaternion3();

      mux_channel(FA);
      gyro_data(&gX_fa, &gY_fa, &gZ_fa);
      accel_data(&aX_fa, &aY_fa, &aZ_fa);
      filter_fa.updateIMU(gX_fa, gY_fa, gZ_fa, aX_fa, aY_fa, aZ_fa);
      sumFw += filter_fa.getQuaternion0();
      sumFx += filter_fa.getQuaternion1();
      sumFy += filter_fa.getQuaternion2();
      sumFz += filter_fa.getQuaternion3();
      
      mux_channel(UA);
      gyro_data(&gX_ua, &gY_ua, &gZ_ua);
      accel_data(&aX_ua, &aY_ua, &aZ_ua);
      filter_ua.updateIMU(gX_ua, gY_ua, gZ_ua, aX_ua, aY_ua, aZ_ua);
      sumUw += filter_ua.getQuaternion0();
      sumUx += filter_ua.getQuaternion1();
      sumUy += filter_ua.getQuaternion2();
      sumUz += filter_ua.getQuaternion3();

      samples++;

      delay(10);
    }

    q0_palm_neutral = sumPw / samples;
    q1_palm_neutral = sumPx / samples;
    q2_palm_neutral = sumPy / samples;
    q3_palm_neutral = sumPz / samples; 

    q0_fa_neutral = sumFw / samples;
    q1_fa_neutral = sumFx / samples;
    q2_fa_neutral = sumFy / samples;
    q3_fa_neutral = sumFz / samples;

    q0_ua_neutral = sumUw / samples;
    q1_ua_neutral = sumUx / samples;
    q2_ua_neutral = sumUy / samples;
    q3_ua_neutral = sumUz / samples;

    save_neutral_pose();
}

void calibrate_flex_rom(){
    Serial.println("Calibrare flex RoM");
    led_wait();

    int start = millis();

    led_hold();
    Serial.println("Miscati degetele pentru RoM a senzorilor de rezistenta");
    led_hold();

    while(millis() - start < FLEX_REFERENCE_TIME){
      get_flex_values();
    }

    save_flex_range();
}

void print_flex_rom(){
    Serial.println("== RoM senzori de rezistenta salvat ==");

    Serial.print("== Thumb Sensor RoM: Min = ");
    Serial.print(thumb_min);
    Serial.print(", Max = ");
    Serial.print(thumb_max);
    Serial.println(" ==");

    Serial.print("== Index Sensor RoM: Min = ");
    Serial.print(index_min);
    Serial.print(", Max = ");
    Serial.print(index_max);
    Serial.println(" ==");

    Serial.print("== Little Sensor RoM: Min = ");
    Serial.print(little_min);
    Serial.print(", Max = ");
    Serial.print(little_max);
    Serial.println(" ==");

    Serial.println("== Reporniti alimentarea ==");
}

void save_gyro_offsets(){
  prefs.begin("gyroCal", false);
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
}

void capture_gyro_offsets(){
    Serial.println("== Calibrare flex RoM ==");

    led_wait();
    led_hold();

    Serial.println("== Calibrare senzor palma ==");
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

    save_gyro_offsets();
}

void print_offset(char *segment, float oX, float oY, float oZ){
  Serial.print("== Offset pe X pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.print(oX);
  Serial.println(" ==");

  Serial.print("== Offset pe Y pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.print(oY);
  Serial.println(" ==");

  Serial.print("== Offset pe Z pentru ");
  Serial.print(segment);
  Serial.print(" este: ");
  Serial.print(oZ);
  Serial.println(" ==");
}

void print_gyro_offsets(){
  print_offset("palma", offX_palm, offY_palm, offZ_palm);
  print_offset("antebrat", offX_fa, offY_fa, offZ_fa);
  print_offset("brat", offX_ua, offY_ua, offZ_ua);
}

void capture_palm_pitch_ref(){
  Serial.println("== Capture Palm Pitch Reference ==");
  led_wait();
  led_hold();

  long start = millis();

  while( millis() - start < PALM_ROM_REFERENCE_TIME){
    mux_channel(PALM); //PALM_ROM_REFERENCE_TIME
    gyro_data(&gX_palm, &gY_palm, &gZ_palm);
    accel_data(&aX_palm, &aY_palm, &aZ_palm);

    filter_palm.updateIMU(gX_palm,gY_palm,gZ_palm,aX_palm,aY_palm,aZ_palm);
    float pitch = filter_palm.getPitch();
    float q0 = filter_palm.getQuaternion0();
    float q1 = filter_palm.getQuaternion1();
    float q2 = filter_palm.getQuaternion2();
    float q3 = filter_palm.getQuaternion3();

    if (pitch < pitch_min) {
      pitch_min = pitch;
      pQ0_pitch_min = q0; 
      pQ1_pitch_min = q1;
      pQ2_pitch_min = q2;
      pQ3_pitch_min = q3;
    }
    if (pitch > pitch_max) {
      pitch_max = pitch;
      pQ0_pitch_max = q0; 
      pQ1_pitch_max = q1; 
      pQ2_pitch_max = q2; 
      pQ3_pitch_max = q3; 
    }

    delay(INTERVAL);
  } 

  Serial.printf("== Pitch  min: %.1f° → Q[%.3f,%.3f,%.3f,%.3f] ==\n",
                pitch_min,
                pQ0_pitch_min,pQ1_pitch_min,pQ2_pitch_min,pQ3_pitch_min);
  Serial.printf("== Pitch  max: %.1f° → Q[%.3f,%.3f,%.3f,%.3f] ==\n",
                pitch_max,
                pQ0_pitch_max, pQ1_pitch_max, pQ2_pitch_max, pQ3_pitch_max);

  prefs.begin("palmPitchRef", false);
  prefs.putFloat("minPitch", pitch_min);
  prefs.putFloat("pQ0min", pQ0_pitch_min);
  prefs.putFloat("pQ1min", pQ1_pitch_min);
  prefs.putFloat("pQ2min", pQ2_pitch_min);
  prefs.putFloat("pQ3min", pQ3_pitch_min);
  prefs.putFloat("maxPitch", pitch_max);
  prefs.putFloat("pQ0max", pQ0_pitch_max);
  prefs.putFloat("pQ1max", pQ1_pitch_max);
  prefs.putFloat("pQ2max", pQ2_pitch_max);
  prefs.putFloat("pQ3max", pQ3_pitch_max);
  prefs.putBool ("captured", true);
  prefs.end();
}

void capture_palm_roll_ref(){
  Serial.println("== Capture Palm Roll Reference ==");
  led_wait();
  led_hold();

  long start = millis();

  while(millis() - start < PALM_ROM_REFERENCE_TIME){
    mux_channel(PALM);
    gyro_data(&gX_palm,&gY_palm,&gZ_palm);
    accel_data(&aX_palm,&aY_palm,&aZ_palm);

    filter_palm.updateIMU(gX_palm,gY_palm,gZ_palm,aX_palm,aY_palm,aZ_palm);
    float roll = filter_palm.getRoll();
    float q0 = filter_palm.getQuaternion0();
    float q1 = filter_palm.getQuaternion1();
    float q2 = filter_palm.getQuaternion2();
    float q3 = filter_palm.getQuaternion3();

    if (roll < roll_min) {
      roll_min = roll;
      pQ0_roll_min = q0; 
      pQ1_roll_min = q1;
      pQ2_roll_min = q2;
      pQ3_roll_min = q3;
    }
    if (roll > roll_max) {
      roll_max = roll;
      pQ0_roll_max = q0;
      pQ1_roll_max = q1;
      pQ2_roll_max = q2;
      pQ3_roll_max = q3;
    }
    delay(INTERVAL);
  }

   Serial.printf("Roll   min: %.1f° → Q[%.3f,%.3f,%.3f,%.3f]\n",
                roll_min,
                pQ0_roll_min,pQ1_roll_min,pQ2_roll_min,pQ3_roll_min);
  Serial.printf("Roll   max: %.1f° → Q[%.3f,%.3f,%.3f,%.3f]\n",
                roll_max,
                pQ0_roll_max,pQ1_roll_max,pQ2_roll_max,pQ3_roll_max);

                prefs.begin("palmRollRef", false);
  prefs.putFloat("minRoll", roll_min);
  prefs.putFloat("pQ0min", pQ0_roll_min);
  prefs.putFloat("pQ1min", pQ1_roll_min);
  prefs.putFloat("pQ2min", pQ2_roll_min);
  prefs.putFloat("pQ3min", pQ3_roll_min);
  prefs.putFloat("maxRoll", roll_max);
  prefs.putFloat("pQ0max", pQ0_roll_max);
  prefs.putFloat("pQ1max", pQ1_roll_max);
  prefs.putFloat("pQ2max", pQ2_roll_max);
  prefs.putFloat("pQ3max", pQ3_roll_max);
  prefs.putBool ("captured", true);
  prefs.end();
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  prefs.begin("gyroCal", true);
  bool isCalibrated = prefs.getBool("calibrated", false);
  prefs.end();
  if(!isCalibrated){
    capture_gyro_offsets();
    print_gyro_offsets();
    while(true){}
  }

  // ===== FLEX SENSORS ROM CAPTURE =====
  prefs.begin("flexRange", true);
  bool flexRoM = prefs.getBool("flexRoM", false);
  prefs.end();
  if(!flexRoM){
    calibrate_flex_rom();
    print_flex_rom();
    while(true){};
  }

  // ===== NEUTRAL POSE REFERENCEE CAPTURE =====
  prefs.begin("neutralPose", true);
  bool neutral_pose = prefs.getBool("neutral_pose", false);
  prefs.end();
  if(!neutral_pose){
    capture_neutral_pose();
    print_neutral_pose();
    while(true){};
  }

  // ===== PALM MIN/MAX ROLL/PITCH + QUATERNIONS
  prefs.begin("palmPitchRef", true);
  if (!prefs.getBool("captured", false)) {
    capture_palm_pitch_ref();
    while(true){};
  }
  prefs.end();

  // palm roll
  prefs.begin("palmRollRef", true);
  if (!prefs.getBool("captured", false)) {
    capture_palm_roll_ref();
    while(true){};
  }
  prefs.end();

  // ===== CONFIGURE IMU SENSORS =====
  config_all();

  Serial.println("== Calibrare... ==");

  // ===== LOAD REFERENCE DATA FROM FLASH =====
  load_gyro_offsets();
  load_flex_range();
  load_neutral_pose();
  Serial.println(">>> Senzori calibrati & referinte incarcate <<<");

  // ===== INITIALIZAREA FILTRE MADGWICK =====
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
    print_angles_quaternions("palma ", q0_palm, q1_palm, q2_palm, q3_palm, roll_palm, pitch_palm, yaw_palm);
    
    print_gyro("antebrat", gX_fa, gY_fa, gZ_fa);
    print_accel("antebrat", aX_fa, aY_fa, aZ_fa);
    print_angles_quaternions("antebrat ", q0_fa, q1_fa, q2_fa, q3_fa, roll_fa, pitch_fa, yaw_fa);

    print_gyro("brat", gX_ua, gY_ua, gZ_ua);
    print_accel("brat", aX_ua, aY_ua, aZ_ua);
    print_angles_quaternions("brat ", q0_ua, q1_ua, q2_ua, q3_ua, roll_ua, pitch_ua, yaw_ua);

    last_print = millis();  
  }
}
