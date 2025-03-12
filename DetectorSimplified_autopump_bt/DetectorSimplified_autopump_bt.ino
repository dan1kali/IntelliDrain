///// LIBRARY INCLUSIONS /////
#include <Wire.h> // I2C communication between Arduino and OLED display
#include <Adafruit_GFX.h>     // Adds graphics features to OLED display
#include <Adafruit_SSD1306.h> // Handles communication with hardware for OLED display
#include <HX711_ADC.h> // Communicates with HX711 load cell amplifier module
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h> // Allows for reading and writing to the EEPROM (non-volatile memory). Used for saving calibration values for load cells
#include <SoftwareSerial.h>

#endif // Included only if the platform is ESP8266, ESP32, or AVR-based (like Arduino)

///// PIN DEFINITIONS /////
const int LOADCELL_SCK_PIN = 12; // Sensor serial clock
const int LOADCELL_DOUT_PIN = 11; // Sensor load cell data
const int WEIGHTSCALE1_SCK_PIN = 8; // Weight Scale 1 SCK
const int WEIGHTSCALE1_DOUT_PIN = 7; // Weight Scale 1 DOUT
const int WEIGHTSCALE2_SCK_PIN = 10; // Weight Scale 2 SCK
const int WEIGHTSCALE2_DOUT_PIN = 9; // Weight Scale 2 DOUT
const byte redPin = 6, orangePin = 5, greenPin = 4, whitePin = 3;  // Pins for LEDs
const byte buttonPin = 2;  // Pin for button
const int command_out_pin = 30; //output to flushing Arduino/Users/macbook/Desktop/Arduino/DetectorSimplified_autopump/DetectorSimplified_autopump.ino

///// OLED DISPLAY SETUP /////
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // This specific OLED screen does not need a dedicated reset pin (it uses I2C communication)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Object represents SSD1306 OLED display

///// LOAD CELL SETUP /////
HX711_ADC LoadCell_0(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); // Sensor Load Cell
HX711_ADC LoadCell_1(WEIGHTSCALE1_DOUT_PIN, WEIGHTSCALE1_SCK_PIN); // Weight Scale 1 Load Cell
HX711_ADC LoadCell_2(WEIGHTSCALE2_DOUT_PIN, WEIGHTSCALE2_SCK_PIN); // Weight Scale 2 Load Cell

///// CALIBRATION VARIABLES /////
// EEPROM addresses where calibration values for load cells are stored
const int calVal_eepromAdress_1 = 0; // eeprom address for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom address for calibration value load cell 2 (4 bytes)

///// TIME VARIABLES /////
unsigned long t = 0;
unsigned long startMillis; // To store the start time
const int updateInterval = 954; //How often data is collected in milliseconds *********************************************

///// LOGIC AND LED CONTROL VARIABLES /////
unsigned long orangeLEDStartTime = 0;  // To track when orange LED is turned on
const long orangeLEDTotalDuration = 10000;  // Total orange LED duration (10 seconds)
const long orangeLEDOnDuration = 5000;  // orange LED stays on for the first 5 seconds
const long orangeLEDFlashDuration = 500;  // Flash interval (500ms)
bool orangeLEDActive = false;  // Flag to check if orange LED is active
bool redLEDActive = false;  // Flag to check if red LED is active
int orangeLEDOffCount = 0;  // Counter for how many times the orange LED has been turned off

///// THRESHOLD VARIABLE /////
float threshold = 0.0;  // Default threshold value
bool thresholdPromptDisplayed = false;  // Flag to check if the prompt has been displayed

///// orange LIGHT FLASHING /////
unsigned long flashInterval = 500;  // Flash interval in milliseconds
unsigned long lastFlashTime = 0;    // Store the time of the last flash
bool orangeLEDState = false;          // Track whether orange LED is on or off

///// BT /////
SoftwareSerial HM10(36, 38); // RX = 2, TX = 3

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// SETUP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void setup() {
  ///// SERIAL COMMUNICATION INITIALIZATION /////
  HM10.begin(9600); // set HM10 serial at 9600 baud rate
  Serial.begin(115200); // set HM10 serial at 9600 baud rate

  ///// OLED INITIALIZATION /////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    for (;;); // If OLED initialization fails, halt the program
  }

  display.clearDisplay();

  ///// CALIBRATION VALUES /////
  float calibrationValue_0 = -286.04; // Calibration value for sensor load cell
  float calibrationValue_1 = -200.43; // Calibration value for weight scale 1 load cell
  float calibrationValue_2 = -216.33; // Calibration value for weight scale 2 load cell

  #if defined(ESP8266) || defined(ESP32)
  #endif

  ///// LOAD CELL INITIALIZATION /////
  LoadCell_0.begin();
  LoadCell_1.begin();
  LoadCell_2.begin();

  ///// TARE & STABILIZATION /////
  unsigned long stabilizingtime = 2000; // tare precision can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // tare operation will be performed
  byte loadcell_0_rdy = 0;   byte loadcell_1_rdy = 0;  byte loadcell_2_rdy = 0; 
  while ((loadcell_0_rdy + loadcell_1_rdy + loadcell_2_rdy) < 3) { 
    if (!loadcell_0_rdy) loadcell_0_rdy = LoadCell_0.startMultiple(stabilizingtime, _tare);
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
  }

  ///// CURRENT TIME /////
  startMillis = millis(); // Stores time, in milliseconds, since the Arduino was powered on or reset (current time)

  ///// SET CALIBRATION VALUES /////
  LoadCell_0.setCalFactor(calibrationValue_1); // user set calibration value for sensor (float)
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value for weight scale 1 (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value for weight scale 2 (float)

  ///// LED PIN INITIALIZATION /////
  pinMode(command_out_pin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(orangePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(buttonPin, INPUT); // Configure button pin as input with internal pull-up resistor
  digitalWrite(redPin, HIGH);  
  digitalWrite(orangePin, HIGH);  
  digitalWrite(greenPin, HIGH);  
  digitalWrite(command_out_pin, LOW);  
  digitalWrite(greenPin, LOW); // Turn on green LED to indicate threshold received
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LOOP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void loop() {
  ///// CURRENT TIME ///// 
  unsigned long currentMillis = millis();
  
  static boolean newDataReady = 0; 

  ///// DISPLAY CURRENT LOAD CELL DATA ///// 
  if (!redLEDActive) {  
    if (LoadCell_0.update()) newDataReady = true;
    LoadCell_1.update();
    LoadCell_2.update();
  }

  // Check for button press to turn off red LED
  if (digitalRead(buttonPin) == HIGH) {  
    digitalWrite(redPin, HIGH);  
    redLEDActive = false;  
    digitalWrite(greenPin, LOW);  
    orangeLEDOffCount = 0;  
    delay(200);  
  }

  if (newDataReady) {
    if (millis() > t + updateInterval) {
      float totalTimeinSeconds = (currentMillis - startMillis) / 1000.0;
      float weight0 = LoadCell_0.getData(); 
      float weight1 = LoadCell_1.getData(); 
      float weight2 = LoadCell_2.getData(); 
      updateOLED(totalTimeinSeconds,weight0,weight1,weight2);

      if (redLEDActive) {
        digitalWrite(greenPin, HIGH);  
        digitalWrite(orangePin, HIGH);   
      } else {
        if (!orangeLEDActive) {
          if (weight0 <= threshold) {
            orangeLEDStartTime = millis();  
            orangeLEDActive = true;  
            digitalWrite(greenPin, HIGH);  
          }
        }
      }

      if (orangeLEDActive) {
        unsigned long elapsedTime = millis() - orangeLEDStartTime;

        if (elapsedTime < orangeLEDOnDuration) {
          digitalWrite(orangePin, LOW);  
          digitalWrite(command_out_pin, HIGH); 
        } else if (elapsedTime >= orangeLEDOnDuration && elapsedTime < orangeLEDTotalDuration) {
          if (millis() - lastFlashTime >= flashInterval) {
            orangeLEDState = !orangeLEDState;  
            digitalWrite(orangePin, orangeLEDState ? LOW : HIGH);  
            digitalWrite(command_out_pin, HIGH); 
            lastFlashTime = millis();  
          }
        }

        if (elapsedTime >= orangeLEDTotalDuration) {
          digitalWrite(orangePin, HIGH);  
          digitalWrite(command_out_pin, LOW); 
          orangeLEDActive = false;  
          digitalWrite(greenPin, LOW);  
          orangeLEDOffCount++;  

          if (orangeLEDOffCount >= 2 && !redLEDActive) {
            digitalWrite(redPin, LOW);  
            redLEDActive = true;  
          }
        }
      }

      if (weight0 > threshold && !redLEDActive && !orangeLEDActive) {
        digitalWrite(greenPin, LOW); 
        orangeLEDOffCount = 0;  
      } else {
        digitalWrite(greenPin, HIGH); 
      }

      newDataReady = 0;
      t = millis();
    }
  }
}  

void updateOLED(float totalTimeinSeconds, float weight0, float weight1, float weight2) {
  displayWeight(weight0);
  int int_weight0 = (int)weight0;
  HM10.println(int_weight0);
  Serial.println(int_weight0);

}

void displayWeight(float weight) {
  display.clearDisplay(); 
  display.setTextColor(WHITE); 

  display.setTextSize(1); 
  display.setCursor(0, 10); 
  display.println("Weight:");

  display.setTextSize(2); 
  display.setCursor(0, 30); 
  display.print(weight, 2); 
  display.print(" g"); 

  display.display(); 
}
