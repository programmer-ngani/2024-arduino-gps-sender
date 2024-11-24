/* Important Notice:
The source codes are provided for educational purposes only:
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

The rights to reproduce, modify, and distribute this code
is solely reserved to the rightful owners/developers:
  Marquez Jasper
  Zulueta Berniel
  Raj Kumar Paul
  Salvacion Warren

© November 2024
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <floatToString.h>

// 1000 millisecond or more
#define GPS_READ_INTERVAL 3000
#define RELAY_DELAY 1000

/* Unit Test: Secret Sauce
0 = Test Disabled, Main Program Active
1 = Unit test for Display Module
2 = Unit test for Switches*
3 = Unit test for PIR Sensor Modules
4 = Unit test for GPS Module
5 = Unit test for GSM Module
6 = Unit test for Relay Module**
* SPDT Toggle/Seesaw Switch and SPST Push-to-Make/Button Switch
** Ultrasonic Speaker and Piezoelectric Buzzer
*/
int UNIT_TEST = 0;
bool DISPLAY_ENABLE = true;

// Define SMS message
const char* message = "Unit test for GSM Module: 0";
// Define SIM800L module pins
const int SIM800L_RX = 6;  // RX pin
const int SIM800L_TX = 7;  // TX pin
SoftwareSerial sim800l(SIM800L_TX, SIM800L_RX);

// Define SMS recipient's phone number
//const char* phoneNumber = "+639123456789";
const char* phoneNumber = "+639987654321";
String addMsgIntro = "System triggered.";

// constants won't change. They're used here to set debug msg and pin numbers:
char myString1[] = "Starting...";
char myString2[] = "Press button to send an SMS.";
char myString3[] = "Sending SMS text.";
char myString4[] = "Selected Mode: ";
int auto_polling_interval = 0; // seconds
int manual_press_duration = 0; // seconds
String m = "Selected Mode: ";
String mapLat = "13.453816";
String mapLng = "121.844418";

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
// Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_RESET     -1
//< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_ADDRESS 0x3C
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const int RXPin = 18, TXPin = 19;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);

// the number of the SPDT Toggle/Seesaw Switch pin
const int btnModeA = 5;
// the number of the SPDT Toggle/Seesaw Switch pin
const int btnModeB = 4;
// the number of the COOLEST pin
const int btnModeC = 3;
// the number of the SPST Push-to-Make/Button Switch pin
const int btnManual = 2;
// the pin that the LED is atteched to
int led = 13;
// the pin that the PIR Sensor Module is atteched to
const int PirMotionRightT = 31;
const int PirMotionRightB = 33;
const int PirMotionLeftT = 35;
const int PirMotionLeftB = 37;

// variables will change:
// variable for reading the SPDT Toggle/Seesaw Switch status
int btnStateA = 1;
// variable for reading the SPDT Toggle/Seesaw Switch status
int btnStateB = 1;
// variable for reading the COOLEST status
int btnStateC = 1;
// variable for reading the SPST Push-to-Make/Button Switch status
int btnStateM = 1;
// by default, no motion detected
int PirStateRightT = LOW;
int PirValRightT = 0;
// by default, no motion detected
int PirStateRightB = LOW;
int PirValRightB = 0;
// by default, no motion detected
int PirStateLeftT = LOW;
int PirValLeftT = 0;
// by default, no motion detected
int PirStateLeftB = LOW;
int PirValLeftB = 0;
// the pin that the relay is atteched to
int relay = 10;

void setup() {
  // put your setup code here, to run once:
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  Serial.begin(9600);
  sim800l.begin(9600);
  if(DISPLAY_ENABLE) {
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // Address 0x3D for 128x64
      Serial.println(F("SSD1306 Display Module not detected."));
      for(;;);
    }
  }

  Serial.println(myString1);
  auto_polling_interval = 1000*auto_polling_interval;
  manual_press_duration = 1000*manual_press_duration;
  if(DISPLAY_ENABLE) {
    ShowOled(myString1, "", 3000);
  }
  pinMode(btnModeA, INPUT_PULLUP);
  pinMode(btnModeB, INPUT_PULLUP);
  pinMode(btnModeC, INPUT_PULLUP);
  pinMode(btnManual, INPUT_PULLUP);
  // initalize LED as an output
  pinMode(led, OUTPUT);
  pinMode(PirMotionRightT, INPUT);
  pinMode(PirMotionRightB, INPUT);
  pinMode(PirMotionLeftT, INPUT);
  pinMode(PirMotionLeftB, INPUT);
  Serial1.begin(GPSBaud);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // Initialize SIM800L module
  sendATCommand("AT", 1000);
  // Once the handshake test is successful, it will back to OK
  sendATCommand("AT+CMGF=1", 1000);  // Set SMS format to text mode
  sendATCommand("AT+CNMI=2,1", 1000);  // Set SMS notification mode
  while(digitalRead(12) == LOW) {
    if(digitalRead(11) == LOW) {
      Serial.println("GPS Search.");
      if(DISPLAY_ENABLE) {
        ShowOled("GPS Search.", "", 3000);
      }
      ReadGps(GPS_READ_INTERVAL);
      SendSms();
    }else {
      Serial.println("Hold button for one second to search and send GPS.");
    }
    digitalWrite(13, LOW);
    delay(1000);
  }
  Serial.println(m += "None");
  checkRunMode();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Mode Selection
  if(DISPLAY_ENABLE) {
    ShowOled(myString4, "None", auto_polling_interval);
  }
  int attachOnce = 0;
  btnStateA = digitalRead(btnModeA);
  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  while(btnStateA == LOW) {
    btnStateA = digitalRead(btnModeA);
    // Manual - Ultrasonic and Piezoelectric
    String m1 = m += "Mode 1";
    Serial.println(m1);
    if(DISPLAY_ENABLE) {
      ShowOled(myString4, "Mode 1", manual_press_duration);
    }
    /*
    if(attachOnce == 0) {
      attachOnce++;
      attachInterrupt(digitalPinToInterrupt(btnManual), SendGps, FALLING);
      Serial.println("Activating manual switch.");
    }
    */
    if(digitalRead(btnManual) == 0) {
      relayControl();
      ReadGps(GPS_READ_INTERVAL);
      SendSms();
    }
  }
  // detachInterrupt(digitalPinToInterrupt(btnManual));
  btnStateB = digitalRead(btnModeB);
  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  while(btnStateB == LOW) {
    btnStateB = digitalRead(btnModeB);
    // Automatic - Ultrasonic and Piezoelectric
    String m2 = m += "Mode 2";
    Serial.println(m2);
    if(DISPLAY_ENABLE) {
      ShowOled(myString4, "Mode 2", auto_polling_interval);
    }
    bool state = CheckPirsFour();
    if(state) {
      relayControl();
      ReadGps(GPS_READ_INTERVAL);
      SendSms();
    }
  }
  btnStateC = digitalRead(btnModeC);
  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  while(btnStateC == LOW) {
    btnStateC = digitalRead(btnModeC);
    // Automatic - Silent Ultrasonic Only
    String m3 = m += "Mode 3";
    Serial.println(m3);
    if(DISPLAY_ENABLE) {
      ShowOled(myString4, "Mode 3", auto_polling_interval);
    }
    bool state = CheckPirsFour();
    if(state) {
      relayControl();
      ReadGps(GPS_READ_INTERVAL);
      SendSms();
    }
  }
}
 
