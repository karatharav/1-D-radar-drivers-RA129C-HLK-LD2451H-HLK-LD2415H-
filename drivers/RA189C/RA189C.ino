//ATMEGA 2560 AND RA189C RADAR INTEGRATION CODE
//READ SPEED DATA & DISPLAY VALUES
//SWITCH LED OFF IF NO OBJECT
//configuratrion setting code WITH ERROR HANDLING
//FULLY_FUNCTIONAL_CODE ACCURACY -+5km/hr
//22/09/25
//EEPROM SAVING
//15/10/25 - tested 
//LED DISPLAY CODE 



//libraries & header files
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <avr/wdt.h>
#include <EEPROM.h>

char buffer[16];    // incoming buffer
char extracted[4];  // to hold HTO + null terminator

//SET speed_threshold, angle, detection_distance
byte frame_0x01[8] = { 0X43, 0X46, 0X01, 0X01, 0X00, 0X14, 0X0D, 0X0A };
//SET detection_direction, sample_frequency, spped_unit
byte frame_0x02[8] = { 0X43, 0X46, 0X02, 0X01, 0X05, 0X00, 0X0D, 0X0A };
//SET anti_interference, speed_datatype
byte frame_0x03[8] = { 0X43, 0X46, 0X03, 0X00, 0X00, 0X01, 0X0D, 0X0A };
//check config_settings
byte frame_0x04[8] = { 0X43, 0X46, 0X04, 0X00, 0X00, 0X00, 0X0D, 0X0A };

byte response_frame[13] = { NULL };  //config_response


int speed_threshold;
int installation_angle;
int detection_distance;
int detection_direction;
int sample_freq;
int speed_unit;
int anti_interference;
int speed_format;
volatile int speed_limit = 40;

const int addr_speed = 0x00;        //0

unsigned long lastDataMillis = 0;            // last time we received sensor data
bool ledsOffDueToTimeout = false;            // true when we've already turned LEDs off
const unsigned long DATA_TIMEOUT_MS = 5000;  // 5 seconds


volatile bool new_data = false;
//user_config_string
char inChar[6] = {};

//EEPROM VARIABLES

char arr[8] = {};
char arr2[8] = {};

char col = 'R';
volatile boolean stringComplete = false;  // whether the string is complete


//rightside
int s11 = 38;  //19        // segment 1
int g = 52;    //18         // second digit
int f = 50;    //17        // second digit
int e = 48;    //16       // second digit
int d = 46;    //15      // second digit
int c = 44;    //14     // second digit

//3pin break

int b = 42;   //13      // second digit
int a = 40;   //12      second digit
int g1 = 54;  //11    3rd digit
int f1 = 56;  //10     3rd digit
int e1 = 58;  //9     3rd digit

//leftside

int d1 = 60;  //8      3rd digit
int c1 = 62;  //7       3rd digit
int G = 64;   //6
int R = 66;   //5

//4PIN BREAK
int b1 = 34;  //4        3rd digit
int a1 = 36;  //3        3rd digit





unsigned long previousMillis = 0;
const long interval = 1000;  // dash blink time

volatile unsigned int hp = 0;
volatile unsigned int tp = 0;
volatile unsigned int op = 0;
int count = 0;
unsigned long tim1 = 0;
unsigned int temp = 120;





void EEPROM_Save(int Data, int addr) {

  EEPROM.update(addr, Data);

  if (EEPROM.read(addr) == Data) {
    Serial.println("BYTE SAVED SUCSESSFULLY");
  } else {
    Serial.println("EEPROM_ERROR");
  }
}



// converting velocity input into display buffer
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

  arr[4] = 0;  //(keeps your existing compare logic working)

  //trigger display update
  stringComplete = true;
}

void all_off() {
  for (int i = 3; i < 20; i++)
    digitalWrite(i, HIGH);
}

void selec(int sel) {
  switch (sel) {
    case 0:
      {
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
        digitalWrite(c, LOW);
        digitalWrite(d, LOW);
        digitalWrite(e, LOW);
        digitalWrite(f, LOW);
        digitalWrite(g, HIGH);
        break;
      }
    case 1:
      {
        digitalWrite(a, HIGH);
        digitalWrite(b, LOW);
        digitalWrite(c, LOW);
        digitalWrite(d, HIGH);
        digitalWrite(e, HIGH);
        digitalWrite(f, HIGH);
        digitalWrite(g, HIGH);
        break;
      }
    case 2:
      {
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
        digitalWrite(c, HIGH);
        digitalWrite(d, LOW);
        digitalWrite(e, LOW);
        digitalWrite(f, HIGH);
        digitalWrite(g, LOW);
        break;
      }
    case 3:
      {
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
        digitalWrite(c, LOW);
        digitalWrite(d, LOW);
        digitalWrite(e, HIGH);
        digitalWrite(f, HIGH);
        digitalWrite(g, LOW);
        break;
      }
    case 4:
      {
        digitalWrite(a, HIGH);
        digitalWrite(b, LOW);
        digitalWrite(c, LOW);
        digitalWrite(d, HIGH);
        digitalWrite(e, HIGH);
        digitalWrite(f, LOW);
        digitalWrite(g, LOW);
        break;
      }
    case 5:
      {
        digitalWrite(a, LOW);
        digitalWrite(b, HIGH);
        digitalWrite(c, LOW);
        digitalWrite(d, LOW);
        digitalWrite(e, HIGH);
        digitalWrite(f, LOW);
        digitalWrite(g, LOW);
        break;
      }
    case 6:
      {
        digitalWrite(a, LOW);
        digitalWrite(b, HIGH);
        digitalWrite(c, LOW);
        digitalWrite(d, LOW);
        digitalWrite(e, LOW);
        digitalWrite(f, LOW);
        digitalWrite(g, LOW);
        break;
      }
    case 7:
      {
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
        digitalWrite(c, LOW);
        digitalWrite(d, HIGH);
        digitalWrite(e, HIGH);
        digitalWrite(f, HIGH);
        digitalWrite(g, HIGH);
        break;
      }
    case 8:
      {
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
        digitalWrite(c, LOW);
        digitalWrite(d, LOW);
        digitalWrite(e, LOW);
        digitalWrite(f, LOW);
        digitalWrite(g, LOW);
        break;
      }
    case 9:
      {
        digitalWrite(a, LOW);
        digitalWrite(b, LOW);
        digitalWrite(c, LOW);
        digitalWrite(d, LOW);
        digitalWrite(e, HIGH);
        digitalWrite(f, LOW);
        digitalWrite(g, LOW);
        break;
      }
  }
}

void selec1(int sel1)  // select 3rd digit
{
  switch (sel1) {
    case 0:
      {
        digitalWrite(a1, LOW);
        digitalWrite(b1, LOW);
        digitalWrite(c1, LOW);
        digitalWrite(d1, LOW);
        digitalWrite(e1, LOW);
        digitalWrite(f1, LOW);
        digitalWrite(g1, HIGH);
        break;
      }
    case 1:
      {
        digitalWrite(a1, HIGH);
        digitalWrite(b1, LOW);
        digitalWrite(c1, LOW);
        digitalWrite(d1, HIGH);
        digitalWrite(e1, HIGH);
        digitalWrite(f1, HIGH);
        digitalWrite(g1, HIGH);
        break;
      }
    case 2:
      {
        digitalWrite(a1, LOW);
        digitalWrite(b1, LOW);
        digitalWrite(c1, HIGH);
        digitalWrite(d1, LOW);
        digitalWrite(e1, LOW);
        digitalWrite(f1, HIGH);
        digitalWrite(g1, LOW);
        break;
      }
    case 3:
      {
        digitalWrite(a1, LOW);
        digitalWrite(b1, LOW);
        digitalWrite(c1, LOW);
        digitalWrite(d1, LOW);
        digitalWrite(e1, HIGH);
        digitalWrite(f1, HIGH);
        digitalWrite(g1, LOW);
        break;
      }
    case 4:
      {
        digitalWrite(a1, HIGH);
        digitalWrite(b1, LOW);
        digitalWrite(c1, LOW);
        digitalWrite(d1, HIGH);
        digitalWrite(e1, HIGH);
        digitalWrite(f1, LOW);
        digitalWrite(g1, LOW);
        break;
      }
    case 5:
      {
        digitalWrite(a1, LOW);
        digitalWrite(b1, HIGH);
        digitalWrite(c1, LOW);
        digitalWrite(d1, LOW);
        digitalWrite(e1, HIGH);
        digitalWrite(f1, LOW);
        digitalWrite(g1, LOW);
        break;
      }
    case 6:
      {
        digitalWrite(a1, LOW);
        digitalWrite(b1, HIGH);
        digitalWrite(c1, LOW);
        digitalWrite(d1, LOW);
        digitalWrite(e1, LOW);
        digitalWrite(f1, LOW);
        digitalWrite(g1, LOW);
        break;
      }
    case 7:
      {
        digitalWrite(a1, LOW);
        digitalWrite(b1, LOW);
        digitalWrite(c1, LOW);
        digitalWrite(d1, HIGH);
        digitalWrite(e1, HIGH);
        digitalWrite(f1, HIGH);
        digitalWrite(g1, HIGH);
        break;
      }
    case 8:
      {
        digitalWrite(a1, LOW);
        digitalWrite(b1, LOW);
        digitalWrite(c1, LOW);
        digitalWrite(d1, LOW);
        digitalWrite(e1, LOW);
        digitalWrite(f1, LOW);
        digitalWrite(g1, LOW);
        break;
      }
    case 9:
      {
        digitalWrite(a1, LOW);
        digitalWrite(b1, LOW);
        digitalWrite(c1, LOW);
        digitalWrite(d1, LOW);
        digitalWrite(e1, HIGH);
        digitalWrite(f1, LOW);
        digitalWrite(g1, LOW);
        break;
      }
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
      digitalWrite(a, HIGH);
      digitalWrite(b, HIGH);
      digitalWrite(c, HIGH);
      digitalWrite(d, HIGH);
      digitalWrite(e, HIGH);
      digitalWrite(f, HIGH);
      digitalWrite(g, HIGH);
    } else
      selec(tp);
  }
  selec1(op);
}




//write the input configuration to RADAR
void Device_Setting(uint8_t *config_frame, const int size, int index, int value, int check_index) {

  bool error_flag = true;
  memset(response_frame, 0XFF, sizeof(response_frame));

  config_frame[index] = (uint8_t)value;
  const int MAX_RETRIES = 5;
  const unsigned long RESPONSE_TIMEOUT_MS = 800; // ms per attempt  int attempts = 0;
  bool success = false; 

  while (error_flag != false) { 
    Serial1.write(config_frame, size);  //writing Command
    delay(100);
    Serial.print("\n Configuration_SENT :  ");
    // print response frame
    for (int i = 0; i < size; i++) {
      Serial.print(config_frame[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");
    delay(300);

    Serial1.write(frame_0x04, sizeof(frame_0x04));  //writing Command
    delay(100);
    Serial.print("\n Configuration_REQUEST :  ");
    // print response frame
    for (int i = 0; i < 8; i++) {
      Serial.print(frame_0x04[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");
    delay(300);



    if (Serial1.available() > 0)  //recived data
    {
      Serial1.readBytes(response_frame, 13); //sizeof(response_frame));
      Serial.print("Configuration_Respone : ");
      delay(100);

      // print response frame

      for (int i = 0; i < 13; i++) {
        Serial.print(response_frame[i], HEX);
        Serial.print(" ");
      }
      Serial.print("\n");

      // check for correct frame header & footer
      if (response_frame[0] == 0x46 && response_frame[12] == 0x0A) {

        // check if radar saved the value correctly
        if (response_frame[check_index] == (uint8_t)value) {
          Serial.println("Setting saved successfully!");
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


  /*

@R_50# → speed_limit = 50

@H_10# → speed_threshold = 10

@A_15# → installation_angle = 15

@D_30# → detection_distance = 30

@N_01# → detection_direction = 1

@F_10# → sample_freq = 10

@U_01# → speed_unit = 1

@I_02# → anti_interference = 2

@P_01# → speed_format = 1

*/


  // final serial_Event() - supports 2 or 3 digit values (e.g. @R_50# or @R_100#)
  // flexible about start marker: accepts formats like "@R_50#", "R_50#", or "@R_100#"
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
        } else {
          Serial.println("speed_limit out of range (0..199)");
        }
        break;

      case 'H':  // speed_threshold
        if (value >= 0 && value <= 220) {
          speed_threshold = value;
          Serial.print("speed_threshold -> ");
          Serial.println(speed_threshold);
          Device_Setting(frame_0x01, sizeof(frame_0x01), 3, speed_threshold, 2);
        } else {
          Serial.println("speed_threshold out of range (0..220)");
        }

        break;

      case 'A':  // installation_angle
        if (value >= 0 && value <= 90) {
          installation_angle = value;
          Serial.print("installation_angle -> ");
          Serial.println(installation_angle);
          Device_Setting(frame_0x01, sizeof(frame_0x01), 4, installation_angle, 3);
        } else {
          Serial.println("installation_angle out of range (0..90)");
        }
        break;

      case 'D':  // detection_distance
        if (value >= 0 && value <= 110) {
          detection_distance = value;
          Serial.print("detection_distance -> ");
          Serial.println(detection_distance);
          Device_Setting(frame_0x01, sizeof(frame_0x01), 5, detection_distance, 4);
        } else {
          Serial.println("detection_distance out of range (0..110)");
        }
        break;

      case 'N':  // detection_direction
        if (value >= 0 && value <= 2) {
          detection_direction = value;
          Serial.print("detection_direction -> ");
          Serial.println(detection_direction);
          Device_Setting(frame_0x02, sizeof(frame_0x02), 3, detection_direction, 5);
        } else {
          Serial.println("detection_direction out of range (0..2)");
        }
        break;

      case 'F':  // sample_freq
        if (value >= 0 && value <= 50) {
          sample_freq = value;
          Serial.print("sample_freq -> ");
          Serial.println(sample_freq);
          Device_Setting(frame_0x02, sizeof(frame_0x02), 4, sample_freq, 6);
        } else {
          Serial.println("sample_freq out of range (0..50)");
        }
        break;

      case 'U':  // speed_unit
        if (value >= 0 && value <= 1) {
          speed_unit = value;
          Serial.print("speed_unit -> ");
          Serial.println(speed_unit);
          Device_Setting(frame_0x02, sizeof(frame_0x02), 5, speed_unit, 7);
        } else {
          Serial.println("speed_unit out of range (0..1)");
        }
        break;

      case 'I':  // anti_interference
        if (value >= 0 && value <= 110) {
          anti_interference = value;
          Serial.print("anti_interference -> ");
          Serial.println(anti_interference);
          Device_Setting(frame_0x03, sizeof(frame_0x03), 3, anti_interference, 8);
        } else {
          Serial.println("anti_interference out of range (0..110)");
        }
        break;

      case 'P':  // speed_format
        if (value >= 0 && value <= 1) {
          speed_format = value;
          Serial.print("speed_format -> ");
          Serial.println(speed_format);
          Device_Setting(frame_0x03, sizeof(frame_0x03), 5, speed_format, 10);
        } else {
          Serial.println("speed_format out of range (0..1)");
        }
        break;

      default:
        Serial.print("Unknown code: ");
        Serial.println(code);
        break;
    }
}


  void setup() {


    Serial.begin(115200);  // USB Serial (PC <-> Arduino)
    Serial1.begin(9600);   // Hardware Serial1 (pins 18=TX1, 19=RX1 -> connect radar here)
    Serial.println("RADAR_INIT");
    delay(100);

        //Read Speed_limit (0x00)
  if (EEPROM.read(addr_speed) != 0xFF) {
    int var = EEPROM.read(addr_speed);
    speed_limit = var;
    Serial.print("/nSPEED_LIMIT : ");
    Serial.print(speed_limit);
    Serial.print(" ");
  }

    int k;
    wdt_disable();
    delay(2L * 1000L);
    wdt_enable(WDTO_2S);

    //pins as output
    for (int pin = 34; pin <= 66; pin += 2) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
    }


    wdt_reset();
    delay(1000);
    wdt_reset();
    delay(1000);
    wdt_reset();
    tim1 = millis();
  }



  void loop() {


    if (Serial1.available()) {

      // Read until newline (sensor sends \r\n at the end)
      int len = Serial1.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
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
                              // Convert to integer
        int velocity = atoi(extracted);

        Serial.print("velocity: ");
        Serial.println(velocity);  // Should print "002"
                                   // Send to your buffer integration function
        integrateVelocityToBuffer((int16_t)velocity);

        // update timestamp and clear the "LEDs off" flag
        lastDataMillis = millis();
        ledsOffDueToTimeout = false;
      }


    } else {
      // no new Serial1 data _right now_ — check timeout
      if ((millis() - lastDataMillis) >= DATA_TIMEOUT_MS) {
        // only turn off LEDs once per timeout event
        if (!ledsOffDueToTimeout) {
          // turn off all segment pins & indicator LEDs
          all_off();             // this sets a bunch of segment pins HIGH (your off state)
          digitalWrite(R, LOW);  // turn off red indicator
          digitalWrite(G, LOW);  // turn off green indicator

          // optionally clear arr so display logic doesn't re-enable segments
          for (int i = 0; i < 5; i++) arr[i] = 0;
          for (int i = 0; i < 5; i++) arr2[i] = 0;

          ledsOffDueToTimeout = true;  // remember we've turned them off
        }
      }
    }




    // put your main code here, to run repeatedly:
    unsigned long tim = millis();
    wdt_reset();
    if (stringComplete == true) {
      stringComplete = false;

      // Serial.println(arr);
      if ((arr[0] != arr2[0]) || (arr[1] != arr2[1]) || (arr[2] != arr2[2]) || (arr[3] != arr2[3]) || (arr[4] != arr2[4]))  // update on new data only
      {
        tim1 = millis() + 5000;
        temp = (((arr[1] - 48) * 100) + ((arr[2] - 48) * 10) + (arr[3] - 48));
        // Serial.println(temp);
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

        // if (col == 'Y') {
        //   all_off();
        //   delay(200);
        // }

        disp_data(temp);



        for (int h = 0; h < 5; h++) { arr2[h] = arr[h]; }
      }
    }



    if (new_data == true) {

      serial_Event();
      //Device_Setting(uint8_t *config_frame, const int size);
      new_data = false;
    }
  }

  void serialEvent() {

    if (Serial.available() > 0) {

      new_data = true;
    }
  }
