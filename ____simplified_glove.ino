#include <Wire.h>
#include "MPU_6050.h"
#include <esp_now.h>
#include <WiFi.h>

//-- DRONE STATUS TRACKING -- 
enum DRONE_STATUS {GROUNDED, TAKING_OFF, AIRBORNE, LANDING};
const char* DRONE_STATUS_NAME[] = {"GROUNDED", "TAKING_OFF", "AIBORNE", "LANDING"};
DRONE_STATUS drone_status;
DRONE_STATUS prev_drone_status;

//-- WIFI --
uint16_t seq;

const uint8_t peerMac[6] = {0x94, 0x54, 0xC5, 0xAF, 0x08, 0x14};

enum MSG_TYPE { SYN, SYN_ACK, ACK, PAYLOAD, INIT_STATUS};

typedef struct {
  uint8_t type;
  uint8_t seq;
  uint8_t command;
  uint latency;
} Packet;

enum STATE { UNINITIALIZED, SENT_SYN, ESTABLISHED } state = UNINITIALIZED;

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len);
void onDataSent(const uint8_t *mac, esp_now_send_status_t stat);

//–– CONFIGURATION ––
unsigned long palmSpikeTs = 0;
const unsigned long PALM_COOLDOWN = 1000;  // ms

float palmEWMA = 0.0f;
const float PALM_ALPHA     = 0.1f;   // small alpha => slow, smooth palm signal
const float PALM_THRESHOLD = 1.10f;

const int PALM_BUF_SZ       = 100;
const float PALM_STEADY_DEG = 0.01f;   // allow ±0.1g around the avg
float palmBuf[PALM_BUF_SZ];
int palmBufPos      = 0;
bool palmBufWarm    = false;

const int    RECALIB_INTERVAL = 200;    // not used for dual-EWMA, but kept for reference
const float  ALPHA_FAST       = 0.2f;   // EWMA alpha for fast tracking (0.0–1.0)
const float  LIT_MIN_PEAK     = 0.15f;  // little finger floor for peak
const float  LIT_MIN_FALL     = 0.08f;  // little finger floor for fall
const float IDX_MIN_PEAK = 0.15f;   // same as little
const float IDX_MIN_FALL = 0.08f;
const float  MIN_ANGLE        = 20.0f;  // degrees for bounce-back
const unsigned long MAX_DT1   = 800;    // ms between spike1 and spike2
const unsigned long MAX_DT2   = 1200;   // ms total for both spikes
const unsigned long PHASE_MS  = 50;     // how long before flipping EWMA phase

//–– MPU INSTANCES ––
MPU_6050 mpuPALM(0x68,0,"PALM"),
        mpuINDX(0x68,1,"INDEX"),
        mpuLITT(0x68,3,"LITTLE");

//-- PALM GESTURES --
enum Direction { FRONT, BACK, LEFT, RIGHT, HOVER, NEUTRAL, TAKE_OFF, LAND };

float FRONT_TH, BACK_TH, LEFT_TH, RIGHT_TH;

Direction direction = NEUTRAL;
Direction lastDirection = NEUTRAL;

void setDirectionThresholds(MPU_6050 &mpu);
void updateDirection(MPU_6050 &mpu);
void printDirection();

//–– CLICK FSM ––
bool pauseMode;
Direction pauseDirection;

enum ClickState { IDLE, SPIKE1, WAIT1, SPIKE2, VALIDATE };
ClickState stI=IDLE, stL=IDLE;
unsigned long t1I,t2I,t1L,t2L;
float a1I,a2I,a1L,a2L;

const unsigned long DOUBLE_CLICK_WINDOW = 1500; //ms
const unsigned long CLICK_COOLDOWN = 400;

unsigned long lastClickTimeI = 0;
uint8_t clickCountI          = 0;

unsigned long lastClickTimeL = 0;
uint8_t clickCountL          = 0; 

//–– INTER-CORE SHARED SAMPLE ––
struct Sample {
  float palmMag;
  float relI, relL;
  float dotI, mag2PI, mag2FI;
  float dotL, mag2PL, mag2FL;
  unsigned long ts;
  Direction dir;
};
static Sample latestSample;
portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;

//–– DUAL-EWMA STATE ––
const float FLT_MAX = 1e6f;
// Index finger
float ewma1_I=0, ewma2_I=0;
float ewma1_min_I=FLT_MAX, ewma1_max_I=-FLT_MAX;
float ewma2_min_I=FLT_MAX, ewma2_max_I=-FLT_MAX;
// Little finger
float ewma1_L=0, ewma2_L=0;
float ewma1_min_L=FLT_MAX, ewma1_max_L=-FLT_MAX;
float ewma2_min_L=FLT_MAX, ewma2_max_L=-FLT_MAX;
// Phase toggling
bool phase = false;                     
unsigned long lastSwitch = 0;

//-- WIFI --
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  Packet p; memcpy(&p, data, sizeof(p));
  if (p.type == SYN_ACK && state==SENT_SYN) {
    Serial.println("← Received SYN_ACK, sending ACK");
    Packet ack = {ACK, p.seq};
    esp_now_send(peerMac, (uint8_t*)&ack, sizeof(ack));
    state = ESTABLISHED;
    Packet p = {INIT_STATUS, seq++, 0};
    esp_now_send(peerMac, (uint8_t*)&p, sizeof(p));
    Serial.println("Handshake complete");
  }


  if(p.type == PAYLOAD && state == ESTABLISHED){
    drone_status = (DRONE_STATUS)p.command;
    if(drone_status != prev_drone_status)
      Serial.printf("%s \n", DRONE_STATUS_NAME[p.command]);
    prev_drone_status = (DRONE_STATUS)p.command;
  }

  if(p.type == INIT_STATUS && state == ESTABLISHED){
    drone_status = (DRONE_STATUS)p.command;
    prev_drone_status = drone_status;
  }
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t stat) {
  if(ESP_NOW_SEND_SUCCESS == FAIL)
    Serial.printf("[send to %02X:%02X:%02X:%02X:%02X:%02X] FAIL\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

//–– UTILITY ––
static inline float calcAngleDeg(float dot, float m2p, float m2f) {
  float c = dot / sqrt(m2p * m2f);
  c = constrain(c, -1.0f, 1.0f);
  return acos(c) * 180.0f/PI;
}

//–– SENSOR TASK (unchanged) ––
void taskSensor(void* pv) {
  Sample tmp;
  while (true) {
    mpuPALM.readAccel(); 
    mpuINDX.readAccel(); 
    mpuLITT.readAccel();
    tmp.palmMag = sqrt(
      sq(mpuPALM.ax_g) +
      sq(mpuPALM.ay_g) +
      sq(mpuPALM.az_g)
    );
    tmp.relI = sqrt(sq(mpuINDX.ax_g-mpuPALM.ax_g)
                  + sq(mpuINDX.ay_g-mpuPALM.ay_g)
                  + sq(mpuINDX.az_g-mpuPALM.az_g));
    tmp.relL = sqrt(sq(mpuLITT.ax_g-mpuPALM.ax_g)
                  + sq(mpuLITT.ay_g-mpuPALM.ay_g)
                  + sq(mpuLITT.az_g-mpuPALM.az_g));
    tmp.dotI   = mpuPALM.ax_g*mpuINDX.ax_g
               + mpuPALM.ay_g*mpuINDX.ay_g
               + mpuPALM.az_g*mpuINDX.az_g;
    tmp.mag2PI = sq(mpuPALM.ax_g)+sq(mpuPALM.ay_g)+sq(mpuPALM.az_g);
    tmp.mag2FI = sq(mpuINDX.ax_g)+sq(mpuINDX.ay_g)+sq(mpuINDX.az_g);
    tmp.dotL   = mpuPALM.ax_g*mpuLITT.ax_g
               + mpuPALM.ay_g*mpuLITT.ay_g
               + mpuPALM.az_g*mpuLITT.az_g;
    tmp.mag2PL = tmp.mag2PI;
    tmp.mag2FL = sq(mpuLITT.ax_g)+sq(mpuLITT.ay_g)+sq(mpuLITT.az_g);
    tmp.ts     = millis();
    updateDirection(mpuPALM);
    if(lastDirection != direction)
      printDirection();
    lastDirection = direction;
    tmp.dir = direction;
    portENTER_CRITICAL(&sampleMux);
      memcpy(&latestSample, &tmp, sizeof(tmp));
    portEXIT_CRITICAL(&sampleMux);
    
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

//–– CLICK FSM FUNCTION & HELPER––
void clickCounter(bool isIndex, Direction dir){
  unsigned long now = millis();

  unsigned long &lastClickTime = isIndex ? lastClickTimeI : lastClickTimeL;
  uint8_t &clickCount = isIndex? clickCountI : clickCountL;

  if(clickCount == 1 && (now - lastClickTime < CLICK_COOLDOWN)) return;

  if(clickCount == 1 && (now - lastClickTime < DOUBLE_CLICK_WINDOW)){
    Serial.printf("✔ %s DOUBLE-CLICK\n", isIndex?"INDEX":"LITTLE");
    delay(1000);
    clickCount = 0;      // reset
    lastClickTime = 0;
    Packet p;
    if(!isIndex){
      if(drone_status == GROUNDED){
        p.type = PAYLOAD;
        p.seq = seq++;
        p.command = TAKE_OFF;
        esp_now_send(peerMac, (uint8_t*) &p, sizeof(p));
      }
      else if (drone_status == AIRBORNE){
        p.type = PAYLOAD;
        p.seq = seq++;
        p.command = LAND;
        esp_now_send(peerMac, (uint8_t*) &p, sizeof(p));
      }
    } else {
      pauseMode = !pauseMode;
      pauseDirection = dir;
    }
  } else {
    clickCount = 1;
    lastClickTime = now;
  }
}

void fsmStep(ClickState& st,
             float rel,
             float palmMag,
             float angle,
             unsigned long& t1,
             unsigned long& t2,
             float& a1,
             float& a2,
             float peak,
             float fall,
             const char* name,
             Direction dir) {
  unsigned long now = millis();
  switch (st) {
    case IDLE:
      if (rel > peak) {
        st = SPIKE1; 
      }
      break;
    case SPIKE1:
      if (rel < fall) {
        t1 = now; a1 = angle; st = WAIT1;
      }
      break;
    case WAIT1:
      if (now - t1 > MAX_DT1) st = IDLE;
      else if (rel > peak) {
        st = SPIKE2;
      }
      break;
    case SPIKE2:
      if (rel < fall) {
        t2 = now; a2 = angle; st = VALIDATE;
      }
      break;
    case VALIDATE: {
      unsigned long dt = t2 - t1;
      float da = fabs(a2 - a1);
      if (dt < MAX_DT2 && da > MIN_ANGLE){
        clickCounter(strcmp(name, "INDEX") == 0, dir);
      }
      st = IDLE;
    } break;
  }
}

//–– SETUP ––
void setup() {
  Serial.begin(115200);
  while(!Serial){}

  seq = 0;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init()!=ESP_OK) { Serial.println("ESP-NOW init fail"); while(1); }
  esp_now_register_recv_cb((esp_now_recv_cb_t)onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer={};
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel=1; peer.encrypt=false;
  esp_now_add_peer(&peer);

  // send SYN
  Packet syn={SYN,0};
  esp_now_send(peerMac,(uint8_t*)&syn,sizeof(syn));
  state = SENT_SYN;
  Serial.println(">> SYN sent");  

  //gestures
  pauseMode = false;

  Wire.begin(21,22);
  mpuPALM.initSensor(300,300,10000);
  mpuINDX.initSensor(0,0,0);
  mpuLITT.initSensor(0,0,0);
  setDirectionThresholds(mpuPALM);
  xTaskCreatePinnedToCore(
    taskSensor, 
    "Sensor", 
    4096, 
    nullptr, 
    1, 
    nullptr, 
    0
  );
  lastSwitch = millis();
}

//–– MAIN LOOP (core 1) ––
void loop() {
  // grab latest preprocessed sample
  uint startGestureDetection  = millis();
  uint finishGestureDetection = millis();
  Sample s;
  portENTER_CRITICAL(&sampleMux);
    memcpy(&s, &latestSample, sizeof(s));
  portEXIT_CRITICAL(&sampleMux);
  float rawPalm = s.palmMag;

  // fill steady‐hand ring buffer
  palmBuf[palmBufPos] = rawPalm;
  if (++palmBufPos >= PALM_BUF_SZ) {
    palmBufPos = 0;
    palmBufWarm = true;
  }

  // compute steady‐hand mean
  float palmMean = rawPalm;
  if (palmBufWarm) {
    palmMean = 0;
    for (int i = 0; i < PALM_BUF_SZ; i++) palmMean += palmBuf[i];
    palmMean /= PALM_BUF_SZ;
  }

  // update slow Palm EWMA
  palmEWMA = PALM_ALPHA * palmEWMA + (1 - PALM_ALPHA) * rawPalm;
  //Serial.printf("PALM raw=%.2fg  EWMA=%.2fg  mean=%.2fg\n",
  //              rawPalm, palmEWMA, palmMean);

  // gate if palm is spiking or unsteady
  if (palmEWMA > PALM_THRESHOLD || fabs(rawPalm - palmMean) > PALM_STEADY_DEG) {
    palmSpikeTs = millis();
    palmBufWarm = false;
    palmBufPos  = 0;
    stI = stL = IDLE;
    return;
  }

  if (millis() - palmSpikeTs < PALM_COOLDOWN) {
    return;
  }

  // compute rel-accels
  float relI = s.relI;
  float relL = s.relL;

  // select which EWMA slot to update this phase
  float &eI    = phase ? ewma2_I    : ewma1_I;
  float &minI  = phase ? ewma2_min_I : ewma1_min_I;
  float &maxI  = phase ? ewma2_max_I : ewma1_max_I;
  float &eL    = phase ? ewma2_L    : ewma1_L;
  float &minL  = phase ? ewma2_min_L : ewma1_min_L;
  float &maxL  = phase ? ewma2_max_L : ewma1_max_L;

  // update EWMA & extrema for this slot
  eI   = ALPHA_FAST * eI + (1 - ALPHA_FAST) * relI;
  minI = min(minI, eI);
  maxI = max(maxI, eI);

  eL   = ALPHA_FAST * eL + (1 - ALPHA_FAST) * relL;
  minL = min(minL, eL);
  maxL = max(maxL, eL);

  // after PHASE_MS, flip phase and reset the newly inactive slot
  if (millis() - lastSwitch >= PHASE_MS) {
    phase = !phase;
    lastSwitch = millis();
    if (phase) {
      ewma1_min_I = FLT_MAX;  ewma1_max_I = -FLT_MAX;
      ewma1_min_L = FLT_MAX;  ewma1_max_L = -FLT_MAX;
    } else {
      ewma2_min_I = FLT_MAX;  ewma2_max_I = -FLT_MAX;
      ewma2_min_L = FLT_MAX;  ewma2_max_L = -FLT_MAX;
    }
  }

  // once both slots have been touched at least once, derive thresholds
  if (lastSwitch > 0) {
    float peakI = max(ewma1_max_I, ewma2_max_I);
    float fallI = min(ewma1_min_I, ewma2_min_I);
    float peakL = max(ewma1_max_L, ewma2_max_L);
    float fallL = min(ewma1_min_L, ewma2_min_L);
    // clamp floors
    peakL = max(peakL, LIT_MIN_PEAK);
    fallL = max(fallL, LIT_MIN_FALL);
    peakI = max(peakI, IDX_MIN_PEAK);
    fallI = max(fallI, IDX_MIN_FALL);

    // compute angles
    float angI = calcAngleDeg(s.dotI, s.mag2PI, s.mag2FI);
    float angL = calcAngleDeg(s.dotL, s.mag2PL, s.mag2FL);

    // run the FSMs
    fsmStep(stI, s.relI, s.palmMag, angI, t1I, t2I, a1I, a2I, peakI, fallI, "INDEX", s.dir);
    fsmStep(stL, s.relL, s.palmMag, angL, t1L, t2L, a1L, a2L, peakL, fallL, "LITTLE", s.dir);
    
    Packet p;
    seq += 1;
    //verificare landing sau taking off -> delay 5 si return
    if(drone_status == TAKING_OFF || drone_status == LANDING){
      delay(5);
      return;
    }

    //verificare grounded -> trimite neutral
    if(drone_status == GROUNDED){
      p.type = PAYLOAD;
      p.seq = seq;
      p.command = s.dir;
      finishGestureDetection = millis();
      p.latency = finishGestureDetection - startGestureDetection;
      esp_now_send(peerMac, (uint8_t*) &p, sizeof(p));
      delay(5);
      return;
    }

    //verificare pauza -> trimitere pauseDirection
    if(pauseMode){
      p.type = PAYLOAD;
      p.seq = seq;
      p.command = pauseDirection;
      finishGestureDetection = millis();
      p.latency = finishGestureDetection - startGestureDetection;
      esp_now_send(peerMac, (uint8_t*) &p, sizeof(p));
      delay(5);
      return;
    }

    //trimite comanda normala -> trimitere comanda curenta
    p.type = PAYLOAD;
    p.seq = seq;
    p.command = s.dir;
    finishGestureDetection = millis();
    p.latency = finishGestureDetection - startGestureDetection;
    esp_now_send(peerMac, (uint8_t*) &p, sizeof(p));
    delay(5);
    return;
  }

  // maintain ~200Hz
  delay(5);
}

// -- PALM GESTURE FUNCTIONS --
void setDirectionThresholds(MPU_6050 &mpu){
  FRONT_TH = (mpu.getMinAX() + mpu.getNeutralAX()) / 2;
  BACK_TH  = (mpu.getMaxAX() + mpu.getNeutralAX()) / 2;
  LEFT_TH  = (mpu.getMinAY() + mpu.getNeutralAY()) / 2;
  RIGHT_TH = (mpu.getMaxAY() + mpu.getNeutralAY()) / 2;
}

void updateDirection(MPU_6050 &mpu){
  uint8_t front = (mpu.ay_g < FRONT_TH) ? 1 : 0; //flexed => negative roll
  uint8_t back  = (mpu.ay_g > BACK_TH) ? 1 : 0;
  uint8_t left  = (mpu.ax_g < LEFT_TH) ? 1 : 0;
  uint8_t right = (mpu.ax_g > RIGHT_TH) ? 1 : 0;

  if(front + back + left + right != 1 && front + back + left + right != 0) return;
  if(drone_status == AIRBORNE){
    if(front + back + left + right == 0) direction = HOVER;

    direction = (front == 1) ? FRONT : direction;
    direction = (back == 1) ? BACK : direction;
    direction = (left == 1) ? LEFT : direction;
    direction = (right == 1) ? RIGHT : direction;
  }

  if(drone_status == GROUNDED) direction = NEUTRAL;
}

void printDirection(){
  switch (direction){
    case FRONT:
      Serial.println("DIRECTION: FRONT");
      break;
    case BACK:
      Serial.println("DIRECTION: BACK");
      break;
    case LEFT:
      Serial.println("DIRECTION: LEFT");
      break;
    case RIGHT:
      Serial.println("DIRECTION: RIGHT");
      break;
    default:
      Serial.println("HOVERING");
      break;
  }
}