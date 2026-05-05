//ATMEGA 2560 AND HLK2415H RADAR INTEGRATION CODE
//ABLE TO READ AND STORE SPEED VALUE
//ABLE TO PROCESS THE HIGHEST SPEED VALUE
//ABLE TO DISPLAY SPEED 
//TURN DISPLAY OFF IF NO DATA
//ACCURACY +-5km/hr
//ABLE TO CONFIGURE SETTINGS
//EEPROM SAVING
//TESTED-6/11/25




#include <Arduino.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>



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

//configuration settings

const int addr_speed = 0x00;        //0

volatile int speed_limit = 40; // threshold to change color to red
int speed_threshold;
int installation_angle;
int sensitivity;
int detection_direction;
int sample_freq;
int speed_unit;
int anti_interference;

volatile bool new_data = false;




unsigned long lastDataMillis = 0;            // last time we received a valid frame
bool ledsOffDueToTimeout = false;            // true when we've already turned LEDs off
const unsigned long DATA_TIMEOUT_MS = 5000;  // 5 seconds




char buffer[16];    // incoming buffer
char extracted[4];  // to hold HTO + null terminator

//SET speed_threshold, angle, sensitivity
byte frame_0x01[8] = { 0X43, 0X46, 0X01, 0X05, 0X00, 0X05, 0X0D, 0X0A };
//SET detection_direction, sample_frequency, speed_unit
byte frame_0x02[8] = { 0X43, 0X46, 0X02, 0X01, 0X00, 0X00, 0X0D, 0X0A };
//SET anti_interference
byte frame_0x03[8] = { 0X43, 0X46, 0X03, 0X00, 0X00, 0X00, 0X0D, 0X0A };
//check config_settings
byte frame_0x04[13] = { 0X43, 0X46, 0X07, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };
//check config_response
//uint8_t response_frame[72]; // keep global as before

#define RESPONSE_FRAME_SIZE 64
uint8_t response_frame[RESPONSE_FRAME_SIZE]; // global response buffer (safe for ATmega)





//noise filterring
const int N = 25;     // maximum samples per second (adjust if needed)
int speed_values[N];  // buffer for velocity readings
int sampleIndex = 0;  // current index in buffer
int speed = 0;        // final max speed for each second
unsigned long time_end = 0;
const unsigned long period = 600;  // 1 second


// -------------------- display driver helpers--------------------

//turn off all leds
void all_off() {
  for (int i = 3; i < 20; i++)
    digitalWrite(i, HIGH);
}

//select leds to turn on
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

//select leds to turn on
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
//divid display input string into parts
unsigned int bifercate(unsigned int xt) {
  hp = (xt / 100);
  tp = ((xt - ((xt / 100) * 100)) / 10);
  op = ((xt - ((xt / 100) * 100)) - ((xt - ((xt / 100) * 100)) / 10) * 10);
}

//turn off leds (fun not in use)
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

//led display driver function
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

//Data saving into EEPROM 
void EEPROM_Save(int Data, int addr) {

  EEPROM.update(addr, Data);

  if (EEPROM.read(addr) == Data) {
    Serial.println("BYTE SAVED SUCSESSFULLY");
  } else {
    Serial.println("EEPROM_ERROR");
  }
}

// -------------------- convert velocity value to display buffer --------------------
void integrateVelocityToBuffer(int16_t velocity) {
  int v = (int)velocity;
  int v_abs = v < 0 ? -v : v;
  if (v_abs > 199) v_abs = 199;

  // color mapping based on speed limit
  if (v_abs >= speed_limit) {
    arr[0] = 'R';
  } else {
    arr[0] = 'G';
  }

  // ASCII digits: hundreds, tens, units
  arr[1] = '0' + (v_abs / 100) % 10;  // hundreds
  arr[2] = '0' + (v_abs / 10) % 10;   // tens
  arr[3] = '0' + (v_abs % 10);        // units

  arr[4] = 0;  

  // trigger display update
  stringComplete = true;

  // optional debug print (throttle or remove if noisy)
  // Serial.println("data");
}


// Parse only X1..X6 from response buffer (safe for 64-byte AVR buffer).
// Now accepts hex digits (0-9, A-F, a-f) and converts using base 16.
// Xvals must have size >= 7 (index 0..6). Xvals[i] = -1 if missing.
static void parse_X1_to_X6(const uint8_t *buf, size_t len, int Xvals[7]) {
  // init X1..X6
  for (int i = 0; i <= 6; ++i) Xvals[i] = -1;

  // limit copy to RESPONSE_FRAME_SIZE bytes to match your buffer limitation
  size_t n = (len > RESPONSE_FRAME_SIZE) ? RESPONSE_FRAME_SIZE : len;
  char tmp[RESPONSE_FRAME_SIZE + 1]; // 64 bytes + null
  size_t i;
  for (i = 0; i < n; ++i) tmp[i] = (char)buf[i];
  tmp[n] = '\0';

  for (i = 0; i + 2 < n; ++i) {
    if (tmp[i] == 'X' && isdigit((unsigned char)tmp[i+1])) {
      int idx = tmp[i+1] - '0';          // X index 0..9
      if (idx < 1 || idx > 6) continue; // we only care X1..X6

      size_t p = i + 2;
      // require ':' soon after the digit (allow small tolerance)
      if (p < n && tmp[p] == ':') {
        ++p;
      } else {
        // search up to 2 bytes forward for ':'
        size_t q = p;
        while (q < n && q < p + 3 && tmp[q] != ':') q++;
        if (q < n && tmp[q] == ':') p = q + 1;
        else continue; // malformed token
      }

      // skip optional spaces
      while (p < n && tmp[p] == ' ') p++;
      // need at least one hex digit
      if (p >= n || !isxdigit((unsigned char)tmp[p])) continue;

      // read up to 4 hex digits (sane limit)
      size_t dstart = p;
      size_t d = dstart;
      while (d < n && isxdigit((unsigned char)tmp[d]) && (d - dstart) < 4) d++;

      // temporarily null-terminate so strtol can parse
      char saved = tmp[d];
      tmp[d] = '\0';
      long val = strtol(&tmp[dstart], NULL, 16); // BASE 16 here!
      tmp[d] = saved;

      Xvals[idx] = (int)val;

      // advance i to end of digits to avoid reprocessing inside the token
      i = d;
    }
  }
}



// find "No" header in buffer; return index or -1 if not found
static int find_No_header(const uint8_t *buf, size_t len) {
  if (len < 2) return -1;
  for (size_t i = 0; i + 1 < len; ++i) {
    if (buf[i] == 'N' && buf[i+1] == 'o') return (int)i;
  }
  return -1;
}


// write the input configuration to RADAR and verify its saved
// - config_frame: command to write
// - size: length of config_frame
// - index: byte index inside config_frame to set to 'value' (you used this previously)
// - value: value to set and verify
// - check_index: the X number (1..6) that should reflect the saved value in radar response
void Device_Setting(uint8_t *config_frame, const int size, int index, int value, int check_index) {

  bool error_flag = true;
  memset((void*)response_frame, 0xFF, sizeof(response_frame));

  // set requested byte in config frame
  if (index >= 0 && index < size) config_frame[index] = (uint8_t)value;

  const int MAX_RETRIES = 5;
  int attempts = 0;
  bool success = false;


  //try multiple attempts if it does,nt save
  while (error_flag != false && attempts < MAX_RETRIES) {
    attempts++;

    // send config
    Serial2.write(config_frame, size);  //writing Command
    delay(100);
    Serial.print("\n Configuration_SENT :  ");
    for (int i = 0; i < size; i++) {
      Serial.print(config_frame[i], HEX);
      Serial.print(" ");
    }
    Serial.print("\n");
    delay(300);

    // request readback 
    Serial2.write(frame_0x04, sizeof(frame_0x04));  //writing Command
    delay(300);
    Serial.print("\n Configuration_REQUEST :  ");
    for (int i = 0; i < 13; i++) {
      Serial.print(frame_0x04[i], HEX);
      Serial.print(" ");
    }
    Serial.print("\n");

    // small wait for response to arrive
    delay(200);


    
    if (Serial2.available() > 0) {
      // read at most RESPONSE_FRAME_SIZE bytes (safe for ATmega)
      size_t toRead = RESPONSE_FRAME_SIZE;
      size_t actuallyRead = Serial2.readBytes((uint8_t*)response_frame, toRead);

      Serial.print("Configuration_Response (bytes read=");
      Serial.print(actuallyRead);
      Serial.print("): ");
      for (size_t i = 0; i < actuallyRead; ++i) {
        Serial.print((char)response_frame[i]); // print as char for readability
      }
      Serial.print("\n");

      // find ASCII "No" start (header can be shifted by non-printables)
      int start = find_No_header(response_frame, actuallyRead);
      if (start >= 0) {
        size_t ascii_len = actuallyRead - (size_t)start;
        size_t parse_len = (ascii_len > RESPONSE_FRAME_SIZE) ? RESPONSE_FRAME_SIZE : ascii_len;

        Serial.print("Found 'No' at index ");
        Serial.print(start);
        Serial.print(", parsing ");
        Serial.print(parse_len);
        Serial.println(" bytes.");

        // parse only from the 'No' onward
        int Xvals[7];
        parse_X1_to_X6(response_frame + start, parse_len, Xvals);

        // print parsed X1..X6
        Serial.println("Parsed X1..X6:");
        for (int xi = 1; xi <= 6; ++xi) {
          Serial.print("X"); Serial.print(xi); Serial.print(" = ");
          if (Xvals[xi] >= 0) Serial.println(Xvals[xi]);
          else Serial.println("-");
        }

        // verify saved value: check_index is the X number (1..6)
        if (check_index >= 1 && check_index <= 6) {
          if (Xvals[check_index] != -1 && Xvals[check_index] == value) {
            Serial.println("Setting saved successfully !");
            error_flag = false;
            success = true;
            break;
          } else {
            Serial.print("Setting NOT saved or mismatch.");
            Serial.print(check_index);
            Serial.print(" = ");
            if (Xvals[check_index] >= 0) Serial.println(Xvals[check_index]);
            else Serial.println("not present");
            error_flag = true; // retry
          }
        } else {
          // fallback: header OK but no valid check_index — assume saved
          Serial.println("Header OK but no valid check_index provided.");
          error_flag = false;
          success = true;
          break;
        }

      } else {
        Serial.println("Header 'No' not found in received bytes. Retrying...");
        error_flag = true;
      }

    } else {
      Serial.println("No response available from Serial2, retrying...");
      error_flag = true;
    }

    delay(200);
  } // end while

  if (!success) {
    Serial.println("Failed to save setting after retries.");
  }
}


// serial_Event() - supports 2 or 3 digit values (e.g. @R_50# or @R_100#)
// flexible about start marker: accepts formats like "@R_50#","@R_100#"
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
        if (value >= 0 && value <= 199) {
          speed_limit = value;
          Serial.print("speed_limit set -> ");
          Serial.println(speed_limit); 
          EEPROM_Save(speed_limit, addr_speed);
          delay(100);
        } else {
          Serial.println("speed_limit out of range (0..199)");
        }
        break;

      case 'H':  // speed_threshold
        if (value >= 0 && value <= 199) {
          speed_threshold = value;
          Serial.print("speed_threshold -> ");
          Serial.println(speed_threshold);
          Device_Setting(frame_0x01, sizeof(frame_0x01), 3, speed_threshold, 1);
        } else {
          Serial.println("speed_threshold out of range (0..199)");
        }

        break;

      case 'A':  // installation_angle
        if (value >= 0 && value <= 45) {
          installation_angle = value;
          Serial.print("installation_angle -> ");
          Serial.println(installation_angle);
          Device_Setting(frame_0x01, sizeof(frame_0x01), 4, installation_angle, 2);
        } else {
          Serial.println("installation_angle out of range (0..45)");
        }
        break;

      case 'D':  // sensitivity
        if (value >= 1 && value <= 15) {
          sensitivity = value;
          Serial.print("sensitivity -> ");
          Serial.println(sensitivity);
          Device_Setting(frame_0x01, sizeof(frame_0x01), 5, sensitivity, 3);
        } else {
          Serial.println("sensitivity out of range (1..15)");
        }
        break;

      case 'N':  // detection_direction
        if (value >= 0 && value <= 2) {
          detection_direction = value;
          Serial.print("detection_direction -> ");
          Serial.println(detection_direction);
          Device_Setting(frame_0x02, sizeof(frame_0x02), 3, detection_direction, 4);
        } else {
          Serial.println("detection_direction out of range (0..2)");
        }
        break;

      case 'F':  // sample_freq
        if (value >= 0 && value <= 1) {
          sample_freq = value;
          Serial.print("sample_freq -> ");
          Serial.println(sample_freq);
          Device_Setting(frame_0x02, sizeof(frame_0x02), 4, sample_freq, 5);
        } else {
          Serial.println("sample_freq out of range (0..1)");
        }
        break;

      case 'U':  // speed_unit
        if (value >= 0 && value <= 1) {
          speed_unit = value;
          Serial.print("speed_unit -> ");
          Serial.println(speed_unit);
          Device_Setting(frame_0x02, sizeof(frame_0x02), 5, speed_unit, 6);
        } else {
          Serial.println("speed_unit out of range (0..1)");
        }
        break;
/*
      case 'I':  // anti_interference
        if (value >= 0 && value <= 70) {
          anti_interference = value;
          Serial.print("anti_interference -> ");
          Serial.println(anti_interference);
          Device_Setting(frame_0x03, sizeof(frame_0x03), 3, anti_interference, 8);
        } else {
          Serial.println("anti_interference out of range (0..70)");
        }
        break;
*/

      default:
        Serial.print("Unknown code: ");
        Serial.println(code);
        break;
    }
}

void setup() {

  Serial.begin(115200);  //Serial for USB Debugging(PC <-> Arduino) for sending command
  Serial2.begin(9600);   //Serial for radar interface(pins (16=TX1 ,17=RX1) -> connects radar)
  Serial.println("RADAR_INIT");
  delay(100);

    //Read Speed_limit saved in eeprom
  if (EEPROM.read(addr_speed) != 0xFF) {
    int var = EEPROM.read(addr_speed);
    speed_limit = var;
    Serial.print("/nSPEED_LIMIT : ");
    Serial.print(speed_limit);
    Serial.print(" ");
  }
   
  //watchdog setup
  wdt_disable();
  delay(2L * 1000L);
  wdt_enable(WDTO_2S);

  // initialize pins for 7 segment display
  for (int pin = 34; pin <= 66; pin += 2) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  
  //watchdog reset
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

  //existing watchdog + display update logic
  unsigned long tim = millis();
  wdt_reset();


  //read & process radar output
  if (Serial2.available()) {

    // Read until newline (sensor sends \r\n at the end)
    int len = Serial2.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';  // null terminate the string

    // Example buffer: "V+002.34\r"

    if (len >= 6 && buffer[0] == 'V' && buffer[1] == '+') {
      // Copy hundreds, tens, ones
      extracted[0] = buffer[2];
      extracted[1] = buffer[3];
      extracted[2] = buffer[4];
      extracted[3] = '\0';  // null terminator
/*                                  
      Serial.print("Raw Input: ");
      Serial.println(buffer);
      
      Serial.print("Extracted: ");
      Serial.println(extracted); // Should print "002"
*/

      int velocity = atoi(extracted);

       //algorithm to display the greatest speed achieved by object in a time period  
      if (sampleIndex < N) {
        speed_values[sampleIndex] = velocity;
        sampleIndex++;
      }

      if (millis() - time_end >= period) {
        time_end = millis();

        if (sampleIndex > 0) {
          // find max
          int maxVal = speed_values[0];
          for (int i = 1; i < sampleIndex; i++) {
            if (speed_values[i] > maxVal) {
              maxVal = speed_values[i];
            }
          }
          speed = maxVal;  // store result
        } else {
          speed = 0;  // no data collected
        }
        velocity = speed;


        // Print result
        Serial.print("Max speed in last scan = ");
        Serial.println(velocity);

        // --- Step 3: Reset array for next second ---
        sampleIndex = 0;
        //clear array values
        memset(speed_values, 0, sizeof(speed_values));

        //create speed string for display 
        integrateVelocityToBuffer((int16_t)velocity);
        
        // update timestamp and clear the "LEDs off" flag
        lastDataMillis = millis();
        ledsOffDueToTimeout = false;
      }


    }
  }
 
  //display trigger
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
      
      //led display driver function 
      disp_data(temp);

      for (int h = 0; h < 5; h++) { arr2[h] = arr[h]; }
    }
  }

  
  // TIMEOUT / LED OFF logic -- BASED ON lastDataMillis
  if ((millis() - lastDataMillis) >= DATA_TIMEOUT_MS) {
    // only perform turn-off once per timeout
    if (!ledsOffDueToTimeout) {
      // turn off all segment pins & indicator LEDs
      all_off();             // this sets a bunch of segment pins HIGH (your off state)
      digitalWrite(R, LOW);  // turn off red indicator
      digitalWrite(G, LOW);  // turn off green indicator

      //clear arr so display logic doesn't re-enable segments
      for (int i = 0; i < 5; i++) arr[i] = 0;
      for (int i = 0; i < 5; i++) arr2[i] = 0;

      ledsOffDueToTimeout = true;  //remember we've turned them off
      Serial.println("data not received - LEDs turned off (timeout)");
    }
  } else {
    // fresh data received recently -> ensure we re-enable indicators on next display update
    ledsOffDueToTimeout = false;
  }

  //small delay to avoid hogging CPU
  delay(5);

     //inturrupt for configuration commands
    if (new_data == true) {
      
      //Serial.println("user settings");
      serial_Event();
      //Device_Setting(uint8_t *config_frame, const int size);
      new_data = false;
    }


}

//inturrupt to handle configuration commands recieved
void serialEvent() {

    if (Serial.available() > 0) {

      new_data = true;
    }
  }