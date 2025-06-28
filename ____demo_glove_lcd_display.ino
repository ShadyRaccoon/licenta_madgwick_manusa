#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // 16x2 LCD with I2C

// Custom characters
byte customMoveChar[] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

byte customHoverChar[] = {
  B11111,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B11111
};

// Drone & connection enums
enum MSG_TYPE { SYN, SYN_ACK, ACK, PAYLOAD, INIT_STATUS };
enum STATE { UNINITIALIZED, SENT_SYN_ACK, ESTABLISHED } state = UNINITIALIZED;
enum DRONE_STATUS { GROUNDED, TAKING_OFF, AIRBORNE, LANDING };
enum Direction { FRONT, BACK, LEFT, RIGHT, HOVER, NEUTRAL, TAKE_OFF, LAND };

// Status & control
DRONE_STATUS drone_status = GROUNDED;
uint8_t seq = 0;
const uint8_t peerMac[6] = {0x20, 0x43, 0xA8, 0x64, 0x8B, 0xE4};

// Control flags
bool flagFront = false;
bool flagBack = false;
bool flagLeft = false;
bool flagRight = false;
bool flagHover = false;

int cursorRow = 0;
int cursorCol = 0;
int prevCol = 0;
int prevRow = 0;

// Message struct
typedef struct {
  uint8_t type;
  uint8_t seq;
  uint8_t command;
} Packet;

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < sizeof(Packet)) return;
  Packet p;
  memcpy(&p, data, sizeof(p));

  if (p.type == SYN) {
    Serial.println("← Received SYN, sending SYN_ACK");
    Packet synAck = { SYN_ACK, p.seq, 0 };
    esp_now_send(peerMac, (uint8_t*)&synAck, sizeof(synAck));
    state = SENT_SYN_ACK;
  } else if (p.type == ACK && state == SENT_SYN_ACK) {
    Serial.println("← Received ACK, handshake complete");
    state = ESTABLISHED;
  }

  if (state == ESTABLISHED && p.type == INIT_STATUS) {
    p = { INIT_STATUS, seq++, drone_status };
    esp_now_send(peerMac, (uint8_t*)&p, sizeof(p));
  }

  if (state == ESTABLISHED && p.type == PAYLOAD) {
    Direction cmd = static_cast<Direction>(p.command);
    switch (cmd) {
      case FRONT:
        if (drone_status == AIRBORNE) flagFront = true;
        break;
      case BACK:
        if (drone_status == AIRBORNE) flagBack = true;
        break;
      case LEFT:
        if (drone_status == AIRBORNE) flagLeft = true;
        break;
      case RIGHT:
        if (drone_status == AIRBORNE) flagRight = true;
        break;
      case HOVER:
        if (drone_status == AIRBORNE) flagHover = true;
        break;
      case LAND:
        if (drone_status == AIRBORNE) {
          drone_status = GROUNDED;
          lcd.noBacklight();
          p = { PAYLOAD, seq++, drone_status };
          esp_now_send(peerMac, (uint8_t*)&p, sizeof(p));
        }
        break;
      case TAKE_OFF:
        if (drone_status == GROUNDED) {
          drone_status = AIRBORNE;
          lcd.backlight();
          p = { PAYLOAD, seq++, drone_status };
          esp_now_send(peerMac, (uint8_t*)&p, sizeof(p));
        }
        break;
      case NEUTRAL:
        if (drone_status == GROUNDED) {
          lcd.noBacklight();
        }
        break;
      default:
        break;
    }
  }
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("[send] ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, customMoveChar);
  lcd.createChar(1, customHoverChar);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(byte(0));

  cursorCol = 0;
  cursorRow = 0;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb((esp_now_recv_cb_t)onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Serial.println("Responder ready: waiting for SYN...");
}

void loop() {
  if (flagFront) {
    lcd.setCursor(prevCol, prevRow);
    lcd.print(" ");
    cursorRow = 0;
    lcd.setCursor(cursorCol, cursorRow);
    lcd.write(byte(0));
    flagFront = false;
    prevRow = cursorRow;
    prevCol = cursorCol;
  }
  if (flagBack) {
    lcd.setCursor(prevCol, prevRow);
    lcd.print(" ");
    cursorRow = 1;
    lcd.print(" ");
    lcd.setCursor(cursorCol, cursorRow);
    lcd.write(byte(0));
    flagBack = false;
    prevRow = cursorRow;
    prevCol = cursorCol;
  }
  if (flagLeft) {
    lcd.setCursor(prevCol, prevRow);
    lcd.print(" ");
    cursorCol--;
    if(cursorCol < 0) cursorCol = 15;
    lcd.print(" ");
    lcd.setCursor(cursorCol, cursorRow);
    lcd.write(byte(0));
    flagLeft = false;
    prevRow = cursorRow;
    prevCol = cursorCol;
  }
  if (flagRight) {
    lcd.setCursor(prevCol, prevRow);
    lcd.print(" ");
    cursorCol++;
    cursorCol = cursorCol % 16;
    lcd.print(" ");
    lcd.setCursor(cursorCol, cursorRow);
    lcd.write(byte(0));
    flagRight = false;
    prevRow = cursorRow;
    prevCol = cursorCol;
  }
  if (flagHover) {
    lcd.setCursor(prevCol, prevRow);
    lcd.print(" ");
    lcd.setCursor(cursorCol, cursorRow);
    lcd.write(byte(1));
    flagHover = false;
    prevRow = cursorRow;
    prevCol = cursorCol;
  }

  delay(50);
}
