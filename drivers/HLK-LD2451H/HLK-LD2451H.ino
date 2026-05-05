// Full final sketch (patched): ATMEGA + HLK-LD2451 multi-target parser + tracker + 7-seg display
// Changes: improved timeout logic and lastDataMillis updates to avoid bogus LED turn-off
// and ensure responsiveness/accuracy after "speed turnoff" logic.
// Able to configure settings through terminal
//EEPROM SAVINGS
//LED DISPLAY CODE 

#include <Arduino.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// -------------------- user existing globals (kept mostly unchanged) --------------------
char arr[8] = {0};      // display buffer (color + digits)
char arr2[8] = {0};     // previous display buffer

char col = 'R';
volatile boolean stringComplete = false;  // used by integrateVelocityToBuffer to trigger update

// 7-seg pin mapping (kept from your original)
int s11 = 38;  int g = 52;  int f = 50;  int e = 48;  int d = 46;  int c = 44;
int b = 42;  int a = 40;  int g1 = 54;  int f1 = 56;  int e1 = 58;
int d1 = 60; int c1 = 62; int G = 64; int R = 66;
int b1 = 34; int a1 = 36;

unsigned long previousMillis = 0;
const long interval = 1000;  // dash blink time

volatile unsigned int hp = 0;
volatile unsigned int tp = 0;
volatile unsigned int op = 0;
int count = 0;
unsigned long tim1 = 0;
unsigned int temp = 120;

volatile int speed_limit = 40; // threshold to change color to red
const int addr_speed = 0x00;        //0

unsigned long lastDataMillis = 0;            // last time we received a valid frame
bool ledsOffDueToTimeout = false;            // true when we've already turned LEDs off
const unsigned long DATA_TIMEOUT_MS = 5000;  // 5 seconds


volatile bool new_data = false;
// -------------------- parser constants & structures --------------------
#define HDR0 0xF4
#define HDR1 0xF3
#define HDR2 0xF2
#define HDR3 0xF1
#define TLR0 0xF8
#define TLR1 0xF7
#define TLR2 0xF6
#define TLR3 0xF5


#define RX_BUF_SIZE 256
static uint8_t rxBuf[RX_BUF_SIZE];
static uint16_t rxLen = 0;

//volatile int speed_limit = 40; // threshold to change color to red
int Dstance_threshold;
int Direction_set;
int Speed_threshold;
int detection_Delay;
int Trigger_Count;
int signal_to_Noise;

//---------radar configuration commands
//config mode enable
byte config_enable[14] = { 0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01 };     
byte config_enable_ack[18] = {NULL};

//config set(distance / direction / speed / delay) 
byte config_frame0x0002[16] = { 0XFD, 0XFC, 0XFB, 0XFA, 0X06, 0X00, 0X02, 0X00, NULL, NULL, NULL, NULL, 0X04, 0X03, 0X02, 0X01 };

//config set(trigger / SNR) 
byte config_frame0x0003[16] = { 0XFD, 0XFC, 0XFB, 0XFA, 0X06, 0X00, 0X03, 0X00, NULL, NULL, NULL, NULL, 0X04, 0X03, 0X02, 0X01 };

//CHECK ERROR
byte config_frame_ack[14] = {NULL};

//config mode disable 
byte config_mode_end[12] = { 0XFD, 0XFC, 0XFB, 0XFA, 0X02, 0X00, 0XFE, 0X00, 0X04, 0X03, 0X02, 0X01 };
byte config_mode_end_ack[14] = {NULL};


#define MAX_TARGETS 8
#define SNR_THRESHOLD 8        // tune this to ignore weak returns
#define APPROACHING_FLAG 0x01  // module-specific flag value (adjust if needed)

struct Target {
  uint8_t angle;
  uint8_t distance;
  uint8_t direction;
  uint8_t speed;
  uint8_t snr;
};
Target targets[MAX_TARGETS];
int targetCount = 0;

// ------------------- TRACKER constants & structs -------------------
#define MAX_TRACKS 8
#define ANGLE_TOL_DEG 6         // tolerance to match angle between frames
#define DIST_TOL_M 3            // tolerance to match distance between frames
#define TRACK_TIMEOUT_MS 800    // ms after lastSeen to consider incrementing miss
#define MIN_HITS_TO_CONFIRM 1   // how many hits needed before track considered confirmed
#define MAX_MISSES 3
#define DEBUG_TRACKING true     // set false to disable verbose prints

struct TrackedTarget {
  int id;
  uint8_t angle;
  uint8_t distance;
  uint8_t direction;
  uint8_t speed;
  uint8_t snr;
  unsigned long lastSeen;
  int hits;
  int misses;
  bool alive;
};
static TrackedTarget tracks[MAX_TRACKS];
static int nextTrackID = 1;

// -------------------- forward prototypes --------------------
void all_off();
void selec(int sel);
void selec1(int sel1);
unsigned int bifercate(unsigned int xt);
void dash1();
void disp_data(int j);
void integrateVelocityToBuffer(int16_t velocity);
void readSerial2IntoBuffer();
void processRxBuffer();
void parseFrame(const uint8_t *frame, uint16_t frameLen);
void initTracks();
void updateTracksFromDetections();
int chooseTrackToDisplay();
int angleByteToDeg(uint8_t a);

// -------------------- display helpers (kept intact) --------------------
void all_off() {
  for (int i = 3; i < 20; i++)
    digitalWrite(i, HIGH);
}

void selec(int sel) {
  switch (sel) {
    case 0: { digitalWrite(a, LOW); digitalWrite(b, LOW); digitalWrite(c, LOW); digitalWrite(d, LOW); digitalWrite(e, LOW); digitalWrite(f, LOW); digitalWrite(g, HIGH); break; }
    case 1: { digitalWrite(a, HIGH); digitalWrite(b, LOW); digitalWrite(c, LOW); digitalWrite(d, HIGH); digitalWrite(e, HIGH); digitalWrite(f, HIGH); digitalWrite(g, HIGH); break; }
    case 2: { digitalWrite(a, LOW); digitalWrite(b, LOW); digitalWrite(c, HIGH); digitalWrite(d, LOW); digitalWrite(e, LOW); digitalWrite(f, HIGH); digitalWrite(g, LOW); break; }
    case 3: { digitalWrite(a, LOW); digitalWrite(b, LOW); digitalWrite(c, LOW); digitalWrite(d, LOW); digitalWrite(e, HIGH); digitalWrite(f, HIGH); digitalWrite(g, LOW); break; }
    case 4: { digitalWrite(a, HIGH); digitalWrite(b, LOW); digitalWrite(c, LOW); digitalWrite(d, HIGH); digitalWrite(e, HIGH); digitalWrite(f, LOW); digitalWrite(g, LOW); break; }
    case 5: { digitalWrite(a, LOW); digitalWrite(b, HIGH); digitalWrite(c, LOW); digitalWrite(d, LOW); digitalWrite(e, HIGH); digitalWrite(f, LOW); digitalWrite(g, LOW); break; }
    case 6: { digitalWrite(a, LOW); digitalWrite(b, HIGH); digitalWrite(c, LOW); digitalWrite(d, LOW); digitalWrite(e, LOW); digitalWrite(f, LOW); digitalWrite(g, LOW); break; }
    case 7: { digitalWrite(a, LOW); digitalWrite(b, LOW); digitalWrite(c, LOW); digitalWrite(d, HIGH); digitalWrite(e, HIGH); digitalWrite(f, HIGH); digitalWrite(g, HIGH); break; }
    case 8: { digitalWrite(a, LOW); digitalWrite(b, LOW); digitalWrite(c, LOW); digitalWrite(d, LOW); digitalWrite(e, LOW); digitalWrite(f, LOW); digitalWrite(g, LOW); break; }
    case 9: { digitalWrite(a, LOW); digitalWrite(b, LOW); digitalWrite(c, LOW); digitalWrite(d, LOW); digitalWrite(e, HIGH); digitalWrite(f, LOW); digitalWrite(g, LOW); break; }
  }
}

void selec1(int sel1) {
  switch (sel1) {
    case 0: { digitalWrite(a1, LOW); digitalWrite(b1, LOW); digitalWrite(c1, LOW); digitalWrite(d1, LOW); digitalWrite(e1, LOW); digitalWrite(f1, LOW); digitalWrite(g1, HIGH); break; }
    case 1: { digitalWrite(a1, HIGH); digitalWrite(b1, LOW); digitalWrite(c1, LOW); digitalWrite(d1, HIGH); digitalWrite(e1, HIGH); digitalWrite(f1, HIGH); digitalWrite(g1, HIGH); break; }
    case 2: { digitalWrite(a1, LOW); digitalWrite(b1, LOW); digitalWrite(c1, HIGH); digitalWrite(d1, LOW); digitalWrite(e1, LOW); digitalWrite(f1, HIGH); digitalWrite(g1, LOW); break; }
    case 3: { digitalWrite(a1, LOW); digitalWrite(b1, LOW); digitalWrite(c1, LOW); digitalWrite(d1, LOW); digitalWrite(e1, HIGH); digitalWrite(f1, HIGH); digitalWrite(g1, LOW); break; }
    case 4: { digitalWrite(a1, HIGH); digitalWrite(b1, LOW); digitalWrite(c1, LOW); digitalWrite(d1, HIGH); digitalWrite(e1, HIGH); digitalWrite(f1, LOW); digitalWrite(g1, LOW); break; }
    case 5: { digitalWrite(a1, LOW); digitalWrite(b1, HIGH); digitalWrite(c1, LOW); digitalWrite(d1, LOW); digitalWrite(e1, HIGH); digitalWrite(f1, LOW); digitalWrite(g1, LOW); break; }
    case 6: { digitalWrite(a1, LOW); digitalWrite(b1, HIGH); digitalWrite(c1, LOW); digitalWrite(d1, LOW); digitalWrite(e1, LOW); digitalWrite(f1, LOW); digitalWrite(g1, LOW); break; }
    case 7: { digitalWrite(a1, LOW); digitalWrite(b1, LOW); digitalWrite(c1, LOW); digitalWrite(d1, HIGH); digitalWrite(e1, HIGH); digitalWrite(f1, HIGH); digitalWrite(g1, HIGH); break; }
    case 8: { digitalWrite(a1, LOW); digitalWrite(b1, LOW); digitalWrite(c1, LOW); digitalWrite(d1, LOW); digitalWrite(e1, LOW); digitalWrite(f1, LOW); digitalWrite(g1, LOW); break; }
    case 9: { digitalWrite(a1, LOW); digitalWrite(b1, LOW); digitalWrite(c1, LOW); digitalWrite(d1, LOW); digitalWrite(e1, HIGH); digitalWrite(f1, LOW); digitalWrite(g1, LOW); break; }
  }
}

unsigned int bifercate(unsigned int xt) {
  hp = (xt / 100);
  tp = ((xt - ((xt / 100) * 100)) / 10);
  op = ((xt - ((xt / 100) * 100)) - ((xt - ((xt / 100) * 100)) / 10) * 10);
}

void dash1() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    digitalWrite(R, HIGH);
    digitalWrite(G, HIGH);

    previousMillis = currentMillis;
    if (count == 0) {
      count = 1;
      all_off();
    } else {
      count = 0;
      digitalWrite(g, LOW);
      digitalWrite(g1, LOW);
    }
  }
}

void disp_data(int j) {
  bifercate(j);

  if (hp == 1) {
    digitalWrite(s11, LOW);
    selec(tp);
  } else {
    digitalWrite(s11, HIGH);
    if (tp == 0) {
      digitalWrite(a, HIGH); digitalWrite(b, HIGH); digitalWrite(c, HIGH); digitalWrite(d, HIGH); digitalWrite(e, HIGH); digitalWrite(f, HIGH); digitalWrite(g, HIGH);
    } else selec(tp);
  }
  selec1(op);
}

void EEPROM_Save(int Data, int addr) {

  EEPROM.update(addr, Data);

  if (EEPROM.read(addr) == Data) {
    Serial.println("BYTE SAVED SUCSESSFULLY");
  } else {
    Serial.println("EEPROM_ERROR");
  }
}

// -------------------- integrate velocity to display buffer --------------------
void integrateVelocityToBuffer(int16_t velocity) {
  int v = (int)velocity;
  int v_abs = v < 0 ? -v : v;
  if (v_abs > 199) v_abs = 199;

  // color mapping
  if (v_abs >= speed_limit) {
    arr[0] = 'R';
  } else {
    arr[0] = 'G';
  }

  // ASCII digits: hundreds, tens, units
  arr[1] = '0' + (v_abs / 100) % 10;  // hundreds
  arr[2] = '0' + (v_abs / 10) % 10;   // tens
  arr[3] = '0' + (v_abs % 10);        // units

  arr[4] = 0;  // keep previous compare logic happy

  // trigger display update
  stringComplete = true;

  // IMPORTANT: update lastDataMillis on any valid display or recent detection
 // lastDataMillis = millis();

  // optional debug print (throttle or remove if noisy)
  // Serial.println("data");
}

//enable configuring 
bool Enable_config(uint8_t *config_start, const int size){

   bool config_flag = false;

   memset(config_enable_ack , 0xFF, sizeof(config_enable_ack));
   const int MAX_RETRIES = 5;
   const unsigned long RESPONSE_TIMEOUT_MS = 800; // ms per attempt  int attempts = 0;
   bool success = false;

   while(config_flag != true){
      Serial2.write(config_start ,size);
      delay(100);
      Serial.print("\n ConfigFrame_SENT :  ");
    // print response frame
    for (int i = 0; i < size; i++) {
      Serial.print(config_start[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");
    delay(300);

        if (Serial2.available() > 0)  //recived data
    {
      Serial2.readBytes(config_enable_ack, sizeof(config_enable_ack)); 
      Serial.print("Config_ACK : ");
      delay(100);

      // print response frame

      for (int i = 0; i < 18; i++) {
        Serial.print(config_enable_ack[i], HEX);
        Serial.print(" ");
      }
      Serial.print("\n");


      // check for correct frame header & footer
      if (config_enable_ack[0] == 0xFD && config_enable_ack[17] == 0x01) {

        // check if radar saved the value correctly
        if (config_enable_ack[8] == 0x00 && config_enable_ack[9] == 0x00) {
          Serial.println("CONFIG_MODE_ENABLE");
          config_flag = true;  // exit loop
        } else {
          Serial.println("attempting configMode, retrying...");
          config_flag = false;  // retry sending
        }
      } else {
        Serial.println("Invalid frame received!");
        config_flag = false;  // retry sending
      }

    } 

  }
}

//ending configuring  
bool Disable_config(uint8_t *config_end, const int size){

   bool config_flag = false;

   memset(config_mode_end_ack , 0xFF, sizeof(config_mode_end_ack));
   const int MAX_RETRIES = 5;
   const unsigned long RESPONSE_TIMEOUT_MS = 800; // ms per attempt  int attempts = 0;
   bool success = false;

   while(config_flag != true){
      Serial2.write(config_end ,size);
      delay(100);
      Serial.print("\n ConfigFrame_SENT :  ");
    // print response frame
    for (int i = 0; i < size; i++) {
      Serial.print(config_end[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");
    delay(300);

        if (Serial2.available() > 0)  //recived data
    {
      Serial2.readBytes(config_mode_end_ack, sizeof(config_mode_end_ack)); 
      Serial.print("Config_END_ACK : ");
      delay(100);

      // print response frame

      for (int i = 0; i < 14; i++) {
        Serial.print(config_mode_end_ack[i], HEX);
        Serial.print(" ");
      }
      Serial.print("\n");


      // check for correct frame header & footer
      if (config_mode_end_ack[0] == 0xFD && config_mode_end_ack[13] == 0x01) {

        // check if radar saved the value correctly
        if (config_mode_end_ack[8] == 0x00 && config_mode_end_ack[9] == 0x00) {
          Serial.println("CONFIG_MODE_DISABLE");
          config_flag = true;  // exit loop
        } else {
          Serial.println("attempting ConfigDisable, retrying...");
          config_flag = false;  // retry sending
        }
      } else {
        Serial.println("Invalid frame received!");
        config_flag = false;  // retry sending
      }

    } 

  }
}



//write the input configuration to RADAR
void Device_Setting(uint8_t *config_frame, const int size, int index, int value) {

  bool error_flag = true;
  memset(config_frame_ack, 0XFF, sizeof(config_frame_ack));

  config_frame[index] = (uint8_t)value;
  const int MAX_RETRIES = 5;
  const unsigned long RESPONSE_TIMEOUT_MS = 800; // ms per attempt  int attempts = 0;
  bool success = false; 

  Enable_config(config_enable, sizeof(config_enable));

  while (error_flag != false) { 
    Serial2.write(config_frame, size);  //writing Command
    delay(100);
    Serial.print("\n Configuration_SENT :  ");
    // print response frame
    for (int i = 0; i < size; i++) {
      Serial.print(config_frame[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");
    delay(300);


    if (Serial2.available() > 0)  //recived data
    {
      Serial2.readBytes(config_frame_ack, 14); //sizeof(response_frame));
      Serial.print("Configuration_Respone : ");
      delay(100);

      // print response frame

      for (int i = 0; i < 14; i++) {
        Serial.print(config_frame_ack[i], HEX);
        Serial.print(" ");
      }
      Serial.print("\n");

      // check for correct frame header & footer
      if (config_frame_ack[0] == 0xFD && config_frame_ack[13] == 0x01) {

        // check if radar saved the value correctly
        if (config_frame_ack[8] == 0x00 && config_frame_ack[9] == 0x00) {
          Serial.println("Setting saved successfully!");

          Disable_config(config_mode_end, sizeof(config_mode_end));

          error_flag = false;  // exit loop
        } else {
          Serial.println("Setting NOT saved, retrying...");
          error_flag = true;  // retry sending
        }
      } else {
        Serial.println("Invalid frame received!");
        error_flag = true;  // retry sending
      }
    }
  }
}



void serial_Event() {
    // local buffer large enough for "@X_999#" + null
    char inBuf[12];
    memset(inBuf, 0, sizeof(inBuf));

    // read up to '#' (delimiter not stored in buffer)
    int got = Serial.readBytesUntil('#', inBuf, sizeof(inBuf) - 1);
    if (got <= 0) return;  // nothing read
    inBuf[got] = '\0';     // null-terminate

    // debug dump
    Serial.print("CMD: ");
    Serial.println(inBuf);

    // find underscore separator
    char *sep = strchr(inBuf, '_');
    if (!sep) {
      Serial.println("Bad format: missing '_'");
      return;
    }

    // determine code letter (character immediately before '_')
    if (sep == inBuf) {
      Serial.println("Bad format: no code before '_'");
      return;
    }
    char code = *(sep - 1);

    // numeric part is everything after '_' (until end of buffer)
    char *numStr = sep + 1;
    if (*numStr == '\0') {
      Serial.println("Bad format: no digits after '_'");
      return;
    }

    // validate digits (allow 1..3 digits)
    int digitCount = 0;
    for (char *p = numStr; *p != '\0'; ++p) {
      if (!isDigit(*p)) {
        Serial.println("Bad format: non-digit in numeric part");
        return;
      }
      ++digitCount;
      if (digitCount > 3) {
        Serial.println("Bad format: numeric too long (>3 digits)");
        return;
      }
    }

    // convert to integer (atoi handles "050" -> 50 and "001" -> 1)
    int value = atoi(numStr);

    // Map code letters to variables (adjust ranges as needed)
    switch (code) {
      case 'R':  // speed_limit
        if (value >= 0 && value <= 120) {
          speed_limit = value;
          Serial.print("speed_limit set -> ");
          Serial.println(speed_limit);
          EEPROM_Save(speed_limit, addr_speed); 
        } else {
          Serial.println("speed_limit out of range (0..120)");
        }
        break;

      case 'K':  // Dstance_threshold
        if (value >= 10 && value <= 100) {
          Dstance_threshold = value;
          Serial.print("Dstance_threshold -> ");
          Serial.println(Dstance_threshold);
          Device_Setting(config_frame0x0002, sizeof(config_frame0x0002), 8, Dstance_threshold);
        } else {
          Serial.println("Dstance_threshold out of range (10..100)");
        }

        break;

      case 'L':  // Direction_set
        if (value >= 0 && value <= 2) {
          Direction_set = value;
          Serial.print("Direction_set -> ");
          Serial.println(Direction_set);
          Device_Setting(config_frame0x0002, sizeof(config_frame0x0002), 9, Direction_set);
        } else {
          Serial.println("Direction_set out of range (0..2)");
        }
        break;

      case 'M':  // Speed_threshold
        if (value >= 0 && value <= 120) {
          Speed_threshold = value;
          Serial.print("Speed_threshold -> ");
          Serial.println(Speed_threshold);
          Device_Setting(config_frame0x0002, sizeof(config_frame0x0002), 10, Speed_threshold);
        } else {
          Serial.println("Speed_threshold out of range (0..120)");
        }
        break;

      case 'N':  // detection_Delay
        if (value >= 0 && value <= 30) {
          detection_Delay = value;
          Serial.print("detection_Delay -> ");
          Serial.println(detection_Delay);
          Device_Setting(config_frame0x0002, sizeof(config_frame0x0002), 11, detection_Delay);
        } else {
          Serial.println("detection_Delay out of range (0..30)");
        }
        break;

      case 'O':  // Trigger_Count
        if (value >= 0 && value <= 10) {
          Trigger_Count = value;
          Serial.print("Trigger_Count -> ");
          Serial.println(Trigger_Count);
          Device_Setting(config_frame0x0003, sizeof(config_frame0x0003), 8, Trigger_Count);
        } else {
          Serial.println("Trigger_Count out of range (0..10)");
        }
        break;

      case 'P':  // signal_to_Noise
        if (value >= 0 && value <= 255) {
          signal_to_Noise = value;
          Serial.print("signal_to_Noise -> ");
          Serial.println(signal_to_Noise);
          Device_Setting(config_frame0x0003, sizeof(config_frame0x0003), 9, signal_to_Noise);
        } else {
          Serial.println("signal_to_Noise out of range (0..255)");
        }
        break;


      default:
        Serial.print("Unknown code: ");
        Serial.println(code);
        break;
    }
}





// -------------------- serial buffer reading & processing --------------------
void readSerial2IntoBuffer() {
  while (Serial2.available()) {
    if (rxLen < RX_BUF_SIZE) {
      rxBuf[rxLen++] = (uint8_t)Serial2.read();
    } else {
      // buffer overflow: shift left by 1 and append
      memmove(rxBuf, rxBuf + 1, RX_BUF_SIZE - 1);
      rxBuf[RX_BUF_SIZE - 1] = (uint8_t)Serial2.read();
    }
  }
}

void processRxBuffer() {
  uint16_t i = 0;
  while (rxLen >= 12 && i <= rxLen - 12) { // minimal frame 12 bytes
    if (rxBuf[i] == HDR0 && rxBuf[i+1] == HDR1 && rxBuf[i+2] == HDR2 && rxBuf[i+3] == HDR3) {
      if (i + 6 > rxLen) break; // need length bytes
      uint16_t payloadLen = (uint16_t)rxBuf[i+4] | ((uint16_t)rxBuf[i+5] << 8);
      uint16_t totalFrameLen = 4 + 2 + payloadLen + 4;
      if (totalFrameLen < 12) { i++; continue; }
      if (i + totalFrameLen > rxLen) break; // wait for rest of frame
      uint16_t tailIndex = i + totalFrameLen - 4;
      if (rxBuf[tailIndex] == TLR0 && rxBuf[tailIndex+1] == TLR1 && rxBuf[tailIndex+2] == TLR2 && rxBuf[tailIndex+3] == TLR3) {
        // parse full frame
        parseFrame(rxBuf + i, totalFrameLen);
        // remove consumed bytes
        uint16_t remain = rxLen - (i + totalFrameLen);
        if (remain > 0) memmove(rxBuf, rxBuf + i + totalFrameLen, remain);
        rxLen = remain;
        i = 0;
        continue;
      } else {
        // bad tail; skip header byte and continue
        i++;
        continue;
      }
    } else {
      i++;
    }
  }

  // protect against runaway buffer growth
  if (rxLen > RX_BUF_SIZE - 16) {
    uint16_t keep = 16;
    memmove(rxBuf, rxBuf + rxLen - keep, keep);
    rxLen = keep;
  }
}

// -------------------- frame parsing --------------------
void parseFrame(const uint8_t *frame, uint16_t frameLen) {
  if (frameLen < 12) return;
  uint16_t payloadLen = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
  const uint8_t *payload = frame + 6;
  if (payloadLen < 2) return;
  uint8_t nTargets = payload[0];
  uint8_t alarmInfo = payload[1];
  uint16_t expected = 2 + (uint16_t)nTargets * 5;
  if (payloadLen < expected) return; // truncated

  // We got a valid frame — update lastDataMillis here (important!)
  lastDataMillis = millis();

  targetCount = min((int)nTargets, MAX_TARGETS);
  for (int i = 0; i < targetCount; ++i) {
    uint16_t base = 2 + i * 5;
    targets[i].angle = payload[base + 0];
    targets[i].distance = payload[base + 1];
    targets[i].direction = payload[base + 2];
    targets[i].speed = payload[base + 3];
    targets[i].snr = payload[base + 4];
  }

  // update tracking with fresh detections
  updateTracksFromDetections();

  // choose a stable track to display
  int dispTrack = chooseTrackToDisplay();
  if (dispTrack >= 0) {
    // display speed of chosen track
    int16_t dispSpeed = (int16_t)tracks[dispTrack].speed;
    // optionally mark away targets negative or switch color: here we keep positive
    integrateVelocityToBuffer(dispSpeed);
  } else {
    // no confirmed track: optionally clear display or keep previous
    // For now, do nothing (keep previous display)
  }
}

// -------------------- tracker helpers --------------------
int angleByteToDeg(uint8_t a) {
  // convert module angle byte to signed degree approximation (doc uses value - 0x80)
  return ((int)a) - 0x80;
}

int angDiffDeg(uint8_t a, uint8_t b) {
  int da = angleByteToDeg(a) - angleByteToDeg(b);
  if (da < 0) da = -da;
  return da;
}

int distDiff(uint8_t d1, uint8_t d2) {
  int dd = (int)d1 - (int)d2;
  if (dd < 0) dd = -dd;
  return dd;
}

void initTracks() {
  for (int i = 0; i < MAX_TRACKS; ++i) {
    tracks[i].alive = false;
    tracks[i].id = 0;
    tracks[i].lastSeen = 0;
    tracks[i].hits = 0;
    tracks[i].misses = 0;
  }
  nextTrackID = 1;
}

int findMatchingTrack(int detIdx) {
  int bestIdx = -1;
  int bestScore = 100000;
  for (int t = 0; t < MAX_TRACKS; ++t) {
    if (!tracks[t].alive) continue;
    unsigned long age = millis() - tracks[t].lastSeen;
    if (age > TRACK_TIMEOUT_MS * 3) continue; // skip very old tracks
    int ad = angDiffDeg(targets[detIdx].angle, tracks[t].angle);
    int dd = distDiff(targets[detIdx].distance, tracks[t].distance);
    if (ad <= ANGLE_TOL_DEG && dd <= DIST_TOL_M) {
      int score = ad + dd * 2;
      if (score < bestScore) {
        bestScore = score;
        bestIdx = t;
      }
    }
  }
  return bestIdx;
}

int createTrackFromDetection(int detIdx) {
  for (int t = 0; t < MAX_TRACKS; ++t) {
    if (!tracks[t].alive) {
      tracks[t].alive = true;
      tracks[t].id = nextTrackID++;
      if (nextTrackID > 1000000) nextTrackID = 1;
      tracks[t].angle = targets[detIdx].angle;
      tracks[t].distance = targets[detIdx].distance;
      tracks[t].direction = targets[detIdx].direction;
      tracks[t].speed = targets[detIdx].speed;
      tracks[t].snr = targets[detIdx].snr;
      tracks[t].lastSeen = millis();
      tracks[t].hits = 1;
      tracks[t].misses = 0;
      return t;
    }
  }
  // no free slot: overwrite the oldest track
  int oldestIdx = 0;
  unsigned long oldestTime = millis();
  for (int t = 0; t < MAX_TRACKS; ++t) {
    if (!tracks[t].alive) { oldestIdx = t; break; }
    if (tracks[t].lastSeen < oldestTime) { oldestTime = tracks[t].lastSeen; oldestIdx = t; }
  }
  tracks[oldestIdx].alive = true;
  tracks[oldestIdx].id = nextTrackID++;
  tracks[oldestIdx].angle = targets[detIdx].angle;
  tracks[oldestIdx].distance = targets[detIdx].distance;
  tracks[oldestIdx].direction = targets[detIdx].direction;
  tracks[oldestIdx].speed = targets[detIdx].speed;
  tracks[oldestIdx].snr = targets[detIdx].snr;
  tracks[oldestIdx].lastSeen = millis();
  tracks[oldestIdx].hits = 1;
  tracks[oldestIdx].misses = 0;
  return oldestIdx;
}

void updateTracksFromDetections() {
  bool matchedDet[MAX_TARGETS] = { false };

  // match detections to tracks
  for (int d = 0; d < targetCount; ++d) {
    if (targets[d].snr < SNR_THRESHOLD) continue;
    int match = findMatchingTrack(d);
    if (match >= 0) {
      // update track
      tracks[match].angle = targets[d].angle;
      tracks[match].distance = targets[d].distance;
      tracks[match].direction = targets[d].direction;
      tracks[match].speed = targets[d].speed;
      tracks[match].snr = targets[d].snr;
      tracks[match].lastSeen = millis();
      tracks[match].hits++;
      tracks[match].misses = 0;
      matchedDet[d] = true;
    }
  }

  // create tracks for unmatched detections
  for (int d = 0; d < targetCount; ++d) {
    if (targets[d].snr < SNR_THRESHOLD) continue;
    if (matchedDet[d]) continue;
    createTrackFromDetection(d);
  }

  // age tracks and remove stale ones
  unsigned long now = millis();
  for (int t = 0; t < MAX_TRACKS; ++t) {
    if (!tracks[t].alive) continue;
    if (now - tracks[t].lastSeen > TRACK_TIMEOUT_MS) {
      tracks[t].misses++;
    }
    if (tracks[t].misses > MAX_MISSES) {
      if (DEBUG_TRACKING) {
        Serial.print("Removing track id=");
        Serial.println(tracks[t].id);
      }
      tracks[t].alive = false;
    }
  }

  // debug print tracks
  if (DEBUG_TRACKING) {
    Serial.println("--- Tracks ---");
    for (int t = 0; t < MAX_TRACKS; ++t) {
      if (!tracks[t].alive) continue;
      Serial.print("ID:"); Serial.print(tracks[t].id);
      Serial.print(" ang:"); Serial.print(angleByteToDeg(tracks[t].angle));
      Serial.print(" dist:"); Serial.print(tracks[t].distance);
      Serial.print(" spd:"); Serial.print(tracks[t].speed);
      Serial.print(" snr:"); Serial.print(tracks[t].snr);
      Serial.print(" hits:"); Serial.print(tracks[t].hits);
      Serial.print(" lastSeen(ms):"); Serial.println(millis() - tracks[t].lastSeen);
    }
  }
}

int chooseTrackToDisplay() {
  int bestIdx = -1;
  uint8_t bestSpeed = 0;
  // prefer confirmed approaching tracks
  for (int t = 0; t < MAX_TRACKS; ++t) {
    if (!tracks[t].alive) continue;
    if (tracks[t].hits < MIN_HITS_TO_CONFIRM) continue;
    if (tracks[t].direction == APPROACHING_FLAG) {
      if (tracks[t].speed > bestSpeed) {
        bestSpeed = tracks[t].speed;
        bestIdx = t;
      }
    }
  }
  if (bestIdx >= 0) return bestIdx;
  // fallback: nearest confirmed
  int nearIdx = -1;
  int nearDist = 10000;
  for (int t = 0; t < MAX_TRACKS; ++t) {
    if (!tracks[t].alive) continue;
    if (tracks[t].hits < MIN_HITS_TO_CONFIRM) continue;
    if ((int)tracks[t].distance < nearDist) {
      nearDist = tracks[t].distance;
      nearIdx = t;
    }
  }
  return nearIdx;
}

// -------------------- setup & loop --------------------
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);    // LD2451 connected to Serial2
  delay(100);

      //Read Speed_limit (0x00)
  if (EEPROM.read(addr_speed) != 0xFF) {
    int var = EEPROM.read(addr_speed);
    speed_limit = var;
    Serial.print("/nSPEED_LIMIT : ");
    Serial.print(speed_limit);
    Serial.print(" ");
  }

  wdt_disable();
  delay(2L * 1000L);
  wdt_enable(WDTO_2S);

  // initialize pins (kept original approach)
  for (int pin = 34; pin <= 66; pin += 2) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }

  initTracks();

  wdt_reset();
  delay(1000);
  wdt_reset();
  delay(1000);
  wdt_reset();

  tim1 = millis();
  Serial.println("RADAR_init");
  Serial.print("\n");
}

void loop() {
  // read incoming bytes and process frames
  readSerial2IntoBuffer();
  processRxBuffer();

  // existing watchdog + display update logic
  unsigned long tim = millis();
  wdt_reset();

  if (stringComplete == true) {
    stringComplete = false;

    if ((arr[0] != arr2[0]) || (arr[1] != arr2[1]) || (arr[2] != arr2[2]) || (arr[3] != arr2[3]) || (arr[4] != arr2[4])) {
      tim1 = millis() + 5000;
      temp = (((arr[1] - 48) * 100) + ((arr[2] - 48) * 10) + (arr[3] - 48));
      col = arr[0];
      if (col == 'R') {
        digitalWrite(G, LOW);
        digitalWrite(R, HIGH);
      } else if (col == 'G') {
        digitalWrite(R, LOW);
        digitalWrite(G, HIGH);
      } else if (col == 'Y') {
        digitalWrite(G, HIGH);
        digitalWrite(R, HIGH);
      }

      disp_data(temp);

      for (int h = 0; h < 5; h++) { arr2[h] = arr[h]; }
    }
  }

  // TIMEOUT / LED OFF logic -- BASED ON lastDataMillis (not Serial2.available)
  if ((millis() - lastDataMillis) >= DATA_TIMEOUT_MS) {
    // only perform turn-off once per timeout
    if (!ledsOffDueToTimeout) {
      // turn off all segment pins & indicator LEDs
      all_off();             // this sets a bunch of segment pins HIGH (your off state)
      digitalWrite(R, LOW);  // turn off red indicator
      digitalWrite(G, LOW);  // turn off green indicator

      // optionally clear arr so display logic doesn't re-enable segments
      for (int i = 0; i < 5; i++) arr[i] = 0;
      for (int i = 0; i < 5; i++) arr2[i] = 0;

      ledsOffDueToTimeout = true;  // remember we've turned them off
      Serial.println("data not received - LEDs turned off (timeout)");
    }
  } else {
    // fresh data received recently -> ensure we re-enable indicators on next display update
    ledsOffDueToTimeout = false;
  }



    if (new_data == true) {
      
      serial_Event();
      new_data = false;
    }


  // small delay to avoid hogging CPU
  delay(5);
}

void serialEvent() {

    if (Serial.available() > 0) {

      new_data = true;
    }
}
