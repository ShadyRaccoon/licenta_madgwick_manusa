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

#define PALM 7

#define INTERVAL 10
#define SAMPLE_FREQ 100

#define DEBOUNCE_DELAY 100
#define DOUBLE_CLICK_WINDOW 2000
#define PAUSE_EXIT_HOVER 500

Preferences prefs;

enum MODE{
  MODE_NORMAL,
  MODE_HOVER, 
  MODE_PAUSE,
  MODE_LAND
};

MODE crt_mode = MODE_NORMAL;

bool last_raw_index = false, debounced_index = false;
bool last_raw_little = false, debounced_little = false;

int last_debounce_time_index  = 0;
int last_debounce_time_little = 0;

int index_click_count = 0;
int index_first_time_click = 0;

int little_click_count = 0;
int little_first_time_click = 0;

int pause_exit_time = 0;

int thumb_min = 4095, thumb_max = 0;
int index_min = 4095, index_max = 0;
int little_min = 4095, little_max = 0;

int index_threshold = 2048;
int little_threshold = 2048;

int palm_r = 255, palm_g = 255, palm_b = 255;

float pitch_min = 10000, pitch_max = -10000;
float roll_min = 10000, roll_max = -10000;

float pitch_neutral, roll_neutral;

long last_print = 0;

float gX_palm, gY_palm, gZ_palm;

float offX_palm, offY_palm, offZ_palm;

float aX_palm, aY_palm, aZ_palm;

float roll_palm, pitch_palm;

float pitch_lo_dead, pitch_hi_dead;
float roll_lo_dead, roll_hi_dead;

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

void led_blink(int r, int g, int b){
  unsigned long lastToggle = millis();
  bool on = true;

  while (true) {
    if (millis() - lastToggle >= 300) {
      on = !on;
      lastToggle = millis();
      if (on) {
        led_color(r, g, b);
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

void print_gyro(float gX, float gY, float gZ){
  Serial.print("== Viteza unghiulara pe X pentru este: ");
  Serial.print(gX);
  Serial.println(" ==");

  Serial.print("== Viteza unghiulara pe Y pentru este: ");
  Serial.print(gY);
  Serial.println(" ==");

  Serial.print("== Viteza unghiulara pe Z pentru este: ");
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

void print_accel(float aX, float aY, float aZ){
  Serial.print("== Acceleratia pe X pentru este: ");
  Serial.print(aX);
  Serial.println(" ==");

  Serial.print("== Acceleratia pe Y pentru este: ");
  Serial.print(aY);
  Serial.println(" ==");

  Serial.print("== Acceleratia pe Z pentru este: ");
  Serial.print(aZ);
  Serial.println(" ==");
}

void compute_angles(){
  roll_palm = atan(aY_palm / sqrt(aX_palm * aX_palm + aZ_palm * aZ_palm)) / (PI / 180);
  pitch_palm = atan(aX_palm / sqrt(aY_palm * aY_palm + aZ_palm * aZ_palm)) / (PI / 180);
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
}

void load_gyro_offsets() {
  prefs.begin("gyroCal", true);
  offX_palm = prefs.getFloat("pX", 0.0f);
  offY_palm = prefs.getFloat("pY", 0.0f);
  offZ_palm = prefs.getFloat("pZ", 0.0f);
  prefs.end();
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
    save_gyro_offsets();
}

void print_offset(float oX, float oY, float oZ){
  Serial.print("== Offset pe X pentru este: ");
  Serial.print(oX);
  Serial.println(" ==");

  Serial.print("== Offset pe Y pentru este: ");
  Serial.print(oY);
  Serial.println(" ==");

  Serial.print("== Offset pe Z pentru este: ");
  Serial.print(oZ);
  Serial.println(" ==");
}

void print_gyro_offsets(){
  print_offset(offX_palm, offY_palm, offZ_palm);
}

void save_neutral_pose(){
  prefs.begin("neutralPose", false);
  prefs.putFloat("nPitch", pitch_neutral);
  prefs.putFloat("nRoll", roll_neutral);
  prefs.putBool("neutral_pose", true);
  prefs.end();
}

void load_neutral_pose(){
  prefs.begin("neutralPose", true);
  roll_neutral = prefs.getFloat("nRoll", 0.0f);
  pitch_neutral = prefs.getFloat("nPitch", 0.0f);
  prefs.end();
}

void print_neutral_pose() {
  Serial.println(F("=== Neutral Pose Reference ==="));

  Serial.printf("== Palm Neutral Angles [Roll, Pitch] : [%.3f, %.3f] ==\n", roll_neutral, pitch_neutral);
}

void capture_neutral_pose(){
    Serial.println("Obtinere date de referinta pt pozitie neutra");

    led_wait();

    int start = millis();
    int samples = 0;
    float sumRoll = 0, sumPitch = 0;

    led_hold();

    while(millis() - start < NEUTRAL_REFERENCE_TIME){
      mux_channel(PALM);
      gyro_data(&gX_palm, &gY_palm, &gZ_palm);
      accel_data(&aX_palm, &aY_palm, &aZ_palm);
      compute_angles();

      sumRoll += roll_palm;
      sumPitch += pitch_palm;

      samples++;

      delay(10);
    }

    roll_neutral = sumRoll / samples;
    pitch_neutral = sumPitch / samples;

    save_neutral_pose();
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

    compute_angles();

    if (pitch_palm < pitch_min) {
      pitch_min = pitch_palm;
    }
    if (pitch_palm > pitch_max) {
      pitch_max = pitch_palm;
    }

    delay(INTERVAL);
  } 

  Serial.printf("== Pitch  min: %.1f° ==\n", pitch_min);
  Serial.printf("== Pitch  max: %.1f° ==\n", pitch_max);

  prefs.begin("palmPitchRef", false);
  prefs.putFloat("minPitch", pitch_min);
  prefs.putFloat("maxPitch", pitch_max);
  prefs.putBool ("captured", true);
  prefs.end();
}

void capture_palm_roll_ref(){
  Serial.println("== Capture Palm Roll Reference ==");
  led_wait();
  led_hold();

  long start = millis();

  while( millis() - start < PALM_ROM_REFERENCE_TIME){
    mux_channel(PALM); //PALM_ROM_REFERENCE_TIME
    gyro_data(&gX_palm, &gY_palm, &gZ_palm);
    accel_data(&aX_palm, &aY_palm, &aZ_palm);

    compute_angles();

    if (roll_palm < roll_min) {
      roll_min = roll_palm;
    }
    if (roll_palm > roll_max) {
      roll_max = roll_palm;
    }

    delay(INTERVAL);
  } 

  Serial.printf("== Roll min: %.1f° ==\n", roll_min);
  Serial.printf("== Roll max: %.1f° ==\n", roll_max);

  prefs.begin("palmRollRef", false);
  prefs.putFloat("minRoll", roll_min);
  prefs.putFloat("maxRoll", roll_max);
  prefs.putBool ("captured", true);
  prefs.end();
}


void determine_gesture(){
  //check for pronatio/supination
  // => flexion/extension is in neutral
  bool pitch_neutral_zone = (pitch_palm >= pitch_lo_dead) && (pitch_palm <= pitch_hi_dead);
  bool roll_neutral_zone = (roll_palm >= roll_lo_dead) && (roll_palm <= roll_hi_dead);

  if (!pitch_neutral_zone && roll_neutral_zone){
    if(pitch_palm > pitch_hi_dead){
      Serial.println("FLEXION");
      led_color(255,0,0);
      palm_r = 255;
      palm_g = 0;
      palm_b = 0;
    } else {
      Serial.println("EXTENSION");
      led_color(0,0,255);
      palm_r = 0;
      palm_g = 0;
      palm_b = 255;
    }
  } else if (!roll_neutral_zone && pitch_neutral_zone){
    if(roll_palm > roll_hi_dead){
      Serial.println("PRONATION");
      led_color(0,255,0);
      palm_r = 0;
      palm_g = 255;
      palm_b = 0;
    } else {
      Serial.println("SUPINATION");
      led_color(255,0,255);
      palm_r = 255;
      palm_g = 0;
      palm_b = 255;
    }
  } else if (roll_neutral_zone && pitch_neutral_zone){
    Serial.println("NEUTRAL");
    led_color(255,255,255);
    palm_r = 255;
    palm_g = 255;
    palm_b = 255;
  } else {
    Serial.println("UNKNOWN");
    led_color(0,255,255);
    palm_r = 255;
    palm_g = 255;
    palm_b = 255;
  }
}

void load_palm_reference(){
  prefs.begin("palmRollRef", true);
  roll_min = prefs.getFloat("minRoll", -180);
  roll_max = prefs.getFloat("maxRoll", 180);
  prefs.end();

  prefs.begin("palmPitchRef", true);
  pitch_min = prefs.getFloat("minPitch", -180);
  pitch_max = prefs.getFloat("maxPitch", 180);
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

  // ===== PALM MIN/MAX ROLL/PITCH
  // palm pitch RoM
  prefs.begin("palmPitchRef", true);
  if (!prefs.getBool("captured", false)) {
    capture_palm_pitch_ref();
    while(true){};
  }
  prefs.end();

  // palm roll RoM
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
  load_palm_reference();
  Serial.println(">>> Senzori calibrati & referinte incarcate <<<");

  pitch_lo_dead  = (pitch_neutral + pitch_min) / 2.0f;
  pitch_hi_dead  = (pitch_neutral + pitch_max) / 2.0f;
  roll_lo_dead   = (roll_neutral  + roll_min)  / 2.0f;
  roll_hi_dead   = (roll_neutral  + roll_max)  / 2.0f;

  index_threshold  = (index_min  + index_max ) / 2;
  little_threshold = (little_min + little_max) / 2;
  
  crt_mode = MODE_NORMAL;
}

void loop() {
  // put your main code here, to run repeatedly:

  mux_channel(PALM);
  delay(5);
  gyro_data(&gX_palm, &gY_palm, &gZ_palm);
  accel_data(&aX_palm, &aY_palm, &aZ_palm);
  calibrate_gyro(&gX_palm, &gY_palm, &gZ_palm, offX_palm, offY_palm, offZ_palm);
  compute_angles();

  determine_gesture();

  if( millis() - last_print >= 1000){
    print_gyro(gX_palm, gY_palm, gZ_palm);
    print_accel(aX_palm, aY_palm, aZ_palm);
    Serial.printf("Unghiurile de roratie sunt: [ %.3f , %.3f ]", roll_palm, pitch_palm);

    last_print = millis();  
  }
}