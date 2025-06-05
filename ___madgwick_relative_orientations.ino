#include <Wire.h>
#include <MadgwickAHRS.h>

#define RED_PIN 5
#define GREEN_PIN 23
#define BLUE_PIN 18 

#define MUX_ADDRESS 0x70

#define PALM_CHANNEL 7
#define UA_CHANNEL 2
#define FA_CHANNEL 0

#define GYRO_CALIBRATION_SAMPLES 150

//data  -> rotation rate & acceleration on the axis
float aX_palm, aY_palm, aZ_palm, gX_palm, gY_palm, gZ_palm; //palm  data
float aX_fa, aY_fa, aZ_fa, gX_fa, gY_fa, gZ_fa;          //forearm  data
float aX_ua, aY_ua, aZ_ua, gX_ua, gY_ua, gZ_ua;             //upper arm  data

//orientation quaternion
float q0_palm, q1_palm, q2_palm, q3_palm; //palm quaternion
float q0_fa, q1_fa, q2_fa, q3_fa;         //forearm quaternion
float q0_ua, q1_ua, q2_ua, q3_ua;         //upper arm quaternion

//avg axis measurement error / offsets for gyro 
float offX_palm, offY_palm, offZ_palm;
float offX_fa, offY_fa, offZ_fa;
float offX_ua, offY_ua, offZ_ua;

//filter handler
Madgwick filter_palm, filter_fa, filter_ua;

//function to select multiplexer
void mux_select(uint8_t i2c_bus){
  if (i2c_bus > 7) return;
  Wire.beginTransmission(MUX_ADDRESS);
  Wire.write(1 << i2c_bus);
  Wire.endTransmission();
}

void mpu_config(){
  Wire.beginTransmission(0x68); // Adresa I2C a MPU6050
  Wire.write(0x1A); // Setare LPF pentru accelerometru
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // Setare scală de măsurare pentru accelerometru
  Wire.write(0x1C);
  Wire.write(0x10); // [-8g ; +8g]
  Wire.endTransmission();

  // Configurare MPU6050 pentru giroscop
  Wire.beginTransmission(0x68); // Adresa I2C a MPU6050
  Wire.write(0x1A); // Setare LPF pentru giroscop
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // Setare scală de măsurare pentru giroscop
  Wire.write(0x1B);
  Wire.write(0x08); // [-500deg/s ; +500deg/s]
  Wire.endTransmission();

  // Ieșire din modul de sleep pentru MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Adresa registrului de control al puterii
  Wire.write(0x00); // Dezactivare mod sleep
  Wire.endTransmission();
}

void get_gyro_offsets(uint8_t channel, float &offX, float &offY, float &offZ){
  float x = 0, y = 0, z = 0;
  float aX, aY, aZ, gX, gY, gZ;

  for(int i = 0 ; i < GYRO_CALIBRATION_SAMPLES ; i++){
    get_raw_data(channel, aX, aY, aZ, gX, gY, gZ);
    x += gX;
    y += gY;
    z += gZ;
  }

  offX = x / GYRO_CALIBRATION_SAMPLES;
  offY = y / GYRO_CALIBRATION_SAMPLES;
  offZ = z / GYRO_CALIBRATION_SAMPLES;
}

void calibrate(float &cal_gX, float &cal_gY, float &cal_gZ, float offX, float offY, float offZ){
  cal_gX -= offX;
  cal_gY -= offY;
  cal_gZ -= offZ;
}

void get_raw_data(uint8_t channel, float &aX, float &aY, float &aZ, float &gX, float &gY, float &gZ){
  //citire date accel
  mux_select(channel);
  Wire.beginTransmission(0x68); // Adresa I2C a MPU6050
  Wire.write(0x3B); // Adresa pentru datele de accelerometru
  // 3B - 40 -> accel
  // 41 - 42 -> temp
  // 43 - 48 -> gyro
  if(Wire.endTransmission() != 0){
    Serial.println("Comunicare I2C esuata.");
    return;
  };

  Wire.requestFrom(0x68, 14); // Cerere de date de la MPU6050

  if(Wire.available() < 14){
    Serial.println("Date indisponibile.");
    return;
  }
  
  // Citire date accelerometru
  int16_t raw_aX = Wire.read() << 8 | Wire.read();
  int16_t raw_aY = Wire.read() << 8 | Wire.read();
  int16_t raw_aZ = Wire.read() << 8 | Wire.read();

  aX = float(raw_aX) / 4096;
  aY = float(raw_aY) / 4096;
  aZ = float(raw_aZ) / 4096;

  //skip registri temperatura
  Wire.read();
  Wire.read();

  // Citire date giroscop
  int16_t raw_gX = Wire.read() << 8 | Wire.read();
  int16_t raw_gY = Wire.read() << 8 | Wire.read();
  int16_t raw_gZ = Wire.read() << 8 | Wire.read();

  gX = float(raw_gX) / 65.5;
  gY = float(raw_gY) / 65.5;
  gZ = float(raw_gZ) / 65.5;
}

void print_data(char* segment, float aX, float aY, float aZ, float gX, float gY, float gZ){
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

  Serial.print("Viteza ungiulara pe X pentru ");
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  mux_select(PALM_CHANNEL);
  mpu_config();
  get_gyro_offsets(PALM_CHANNEL, offX_palm, offY_palm, offZ_palm);

  mux_select(FA_CHANNEL);
  mpu_config();
  get_gyro_offsets(FA_CHANNEL, offX_fa, offY_fa, offZ_fa);

  mux_select(UA_CHANNEL);
  mpu_config();
  get_gyro_offsets(UA_CHANNEL, offX_ua, offY_ua, offZ_ua);

  delay(300);
}

void loop() {
  // put your main code here, to run repeatedly:
  get_raw_data(PALM_CHANNEL, aX_palm, aY_palm, aZ_palm, gX_palm, gY_palm, gZ_palm);
  calibrate(gX_palm, gY_palm, gZ_palm, offX_palm, offY_palm, offZ_palm);
  filter_palm.updateIMU(gX_palm, gY_palm, gZ_palm, aX_palm, aY_palm, aZ_palm);
  print_data("palma", aX_palm, aY_palm, aZ_palm, gX_palm, gY_palm, gZ_palm);

  get_raw_data(FA_CHANNEL, aX_fa, aY_fa, aZ_fa, gX_fa, gY_fa, gZ_fa);
  calibrate(gX_fa, gY_fa, gZ_fa, offX_fa, offY_fa, offZ_fa);
  filter_fa.updateIMU(gX_fa, gY_fa, gZ_fa, aX_fa, aY_fa, aZ_fa);
  print_data("antebrat", aX_fa, aY_fa, aZ_fa, gX_fa, gY_fa, gZ_fa);

  get_raw_data(UA_CHANNEL, aX_ua, aY_ua, aZ_ua, gX_ua, gY_ua, gZ_ua);
  calibrate(gX_ua, gY_ua, gZ_ua, offX_ua, offY_ua, offZ_ua);
  filter_fa.updateIMU(gX_ua, gY_ua, gZ_ua, aX_ua, aY_ua, aZ_ua);
  print_data("bratul superior", aX_ua, aY_ua, aZ_ua, gX_ua, gY_ua, gZ_ua);

  delay(500);
}
