///// LIBRARY INCLUSIONS /////
#include <Wire.h> // I2C communication between Arduino and OLED display
#include <Adafruit_GFX.h>     // Adds graphics features to OLED display
#include <Adafruit_SSD1306.h> // Handles communication with hardware for OLED display
#include <HX711_ADC.h> // Communicates with HX711 load cell amplifier module
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h> // Allows for reading and writing to the EEPROM (non-volatile memory). Used for saving calibration values for load cells
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


///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// SETUP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void setup() {
  ///// SERIAL COMMUNICATION INITIALIZATION /////
  Serial.begin(115200);
  Serial.println("Starting...");

  ///// OLED INITIALIZATION /////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    Serial.println("OLED initialization failed");
    for (;;); // If OLED initialization fails, halt the program
  }

  display.clearDisplay();
  Serial.println("OLED initialized"); // OLED display successfully initialized and ready to be use

  ///// CALIBRATION VALUES /////
  float calibrationValue_0 = -286.04; // Calibration value for sensor load cell
  float calibrationValue_1 = -200.43; // Calibration value for weight scale 1 load cell
  float calibrationValue_2 = -216.33; // Calibration value for weight scale 2 load cell

  #if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
  #endif
  //EEPROM.get(calVal_eepromAdress_1, calibrationValue_1); // uncomment this if you want to fetch the value from eeprom
  //EEPROM.get(calVal_eepromAdress_2, calibrationValue_2); // uncomment this if you want to fetch the value from eeprom

  ///// LOAD CELL INITIALIZATION /////
  LoadCell_0.begin();
  LoadCell_1.begin();
  LoadCell_2.begin();

  ///// TARE & STABILIZATION /////
  unsigned long stabilizingtime = 2000; // tare precision can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // tare operation will be performed
  byte loadcell_0_rdy = 0;   byte loadcell_1_rdy = 0;  byte loadcell_2_rdy = 0; // 
  while ((loadcell_0_rdy + loadcell_1_rdy + loadcell_2_rdy) < 3) { 
    if (!loadcell_0_rdy) loadcell_0_rdy = LoadCell_0.startMultiple(stabilizingtime, _tare);
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
  } // Loop continuously attempts to stabilize and tare each load cell until they are all ready
  // Checks whether the tare operation failed due to timing out. If a timeout is detected, an error message displays.
  if (LoadCell_0.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.0 wiring and pin designations (occlusion sensor)"); }
   if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations (weight scale 1)"); }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations (weight scale 2)"); }

  ///// CURRENT TIME /////
  startMillis = millis(); // Stores time, in milliseconds, since the Arduino was powered on or reset (current time) ****************************

  ///// SET CALIBRATION VALUES /////
  LoadCell_0.setCalFactor(calibrationValue_1); // user set calibration value for sensor (float)
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value for weight scale 1 (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value for weight sclae 2 (float)
  Serial.println("Weight scales startup is complete");

  ///// LED PIN INITIALIZATION /////
  pinMode(command_out_pin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(orangePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  //pinMode(whitePin, OUTPUT);
  pinMode(buttonPin, INPUT); // Configure button pin as input with internal pull-up resistor
  // Start with only white LED on during baseline calculation
  //digitalWrite(whitePin, HIGH);
  digitalWrite(redPin, HIGH);  
  digitalWrite(orangePin, HIGH);  
  digitalWrite(greenPin, HIGH);  
  digitalWrite(command_out_pin, LOW);  
  Serial.println("System on!");
  


  /* ///// WAIT FOR THRESHOLD INPUT ///// 
  while (!Serial);  // Wait for the serial port to connect (necessary for some boards)
  
  Serial.println("Enter the threshold value:");
  // Wait for user input to set the threshold value
  while (Serial.available() == 0);  // Wait until data is available
  threshold = Serial.parseFloat();  // Read the threshold value from serial
  
  // Optionally, print the threshold to confirm it has been set
  Serial.print("Threshold set to: ");
  Serial.println(threshold); */

  threshold = -100;

  ///// LED CONTROL AFTER THRESHOLD SET ///// 
  //digitalWrite(whitePin, LOW);  // Turn off white LED
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
  if (!redLEDActive) {  // Only update load cell readings if the red LED is not active
    if (LoadCell_0.update()) newDataReady = true;
    LoadCell_1.update();
    LoadCell_2.update();
  }

  // Check for button press to turn off red LED
  if (digitalRead(buttonPin) == HIGH) {  // Button is pressed (assuming button is connected to ground)
    digitalWrite(redPin, HIGH);  // Turn off red LED
    redLEDActive = false;  // Reset the red LED active flag
    digitalWrite(greenPin, LOW);  // Reactivate green LED
    orangeLEDOffCount = 0;  // Reset the orange LED off counter
    Serial.println("Red LED turned off. Resume.");
    delay(200);  // Simple debounce delay
  }

  if (newDataReady) {
    if (millis() > t + updateInterval) {

    float totalTimeinSeconds = (currentMillis - startMillis) / 1000.0;
    float weight0 = LoadCell_0.getData(); 
    float weight1 = LoadCell_1.getData(); 
    float weight2 = LoadCell_2.getData(); 
    updateOLED(totalTimeinSeconds,weight0,weight1,weight2);

      if (redLEDActive) {
        digitalWrite(greenPin, HIGH);  // Turn off green LED
        digitalWrite(orangePin, HIGH);   // Turn off orange LED

        } 
        
        else {
        // Check if the orange LED is not active (i.e., green LED should remain on)
        if (!orangeLEDActive) {
          if (weight0 <= threshold) {
            orangeLEDStartTime = millis();  // Record the start time
            orangeLEDActive = true;  // Set the flag to true to track the orange LED timing
            digitalWrite(greenPin, HIGH);  // Turn off green LED when orange is activated
          }
        }
      }

      // Logic for handling orange LED and red LED
      if (orangeLEDActive) {
        // Calculate elapsed time since orange LED was turned on
        unsigned long elapsedTime = millis() - orangeLEDStartTime;

        // First 5 seconds: orange LED stays on
        if (elapsedTime < orangeLEDOnDuration) {
          digitalWrite(orangePin, LOW);  // Ensure the orange LED stays on
          digitalWrite(command_out_pin, HIGH); // Send command to start flush pump
          Serial.println("command out pin high");

        }
        // Last 5 seconds: orange LED flashes
        else if (elapsedTime >= orangeLEDOnDuration && elapsedTime < orangeLEDTotalDuration) {
          // Flash orange LED every 500 milliseconds
          if (millis() - lastFlashTime >= flashInterval) {
            orangeLEDState = !orangeLEDState;  // Toggle LED state
            digitalWrite(orangePin, orangeLEDState ? LOW : HIGH);  // Apply the new state
            digitalWrite(command_out_pin, HIGH); // Send command to start flush pump
            Serial.println("command out pin high (flashing)");
            lastFlashTime = millis();  // Update the last flash time
          }
        }

        // After 10 seconds, turn off the orange LED and restore green LED
        if (elapsedTime >= orangeLEDTotalDuration) {
          digitalWrite(orangePin, HIGH);  // Turn off orange LED
          digitalWrite(command_out_pin, LOW); // Send command to start flush pump
          Serial.println("command out pin low");
          orangeLEDActive = false;  // Reset the flag
          digitalWrite(greenPin, LOW);  // Turn green LED back on after orange LED is off
          orangeLEDOffCount++;  // Increment the orange LED off counter

          // After the orange LED has been turned off twice, activate the red LED
          if (orangeLEDOffCount >= 2 && !redLEDActive) {
            digitalWrite(redPin, LOW);  // Turn on red LED
            redLEDActive = true;  // Set flag to track red LED status
            Serial.println("Red LED activated.");
          }
        }
      }

      // ** New logic for green LED to stay off when any other light is on ** 
      if (weight0 > threshold && !redLEDActive && !orangeLEDActive) {
        digitalWrite(greenPin, LOW); // Turn on green LED only if no other LED is active
        orangeLEDOffCount = 0;  // Reset the orange LED off counter
      } else {
        digitalWrite(greenPin, HIGH); // Turn off green LED otherwise
      }

      newDataReady = 0;
      t = millis();

    
    }
  }

  

  /* // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      char secondByte = Serial.read();  // Read the second character for the specific tare operation
      if (secondByte == 'o') {
        LoadCell_0.tareNoDelay();
      }
      else if (secondByte == 's') {
        LoadCell_1.tareNoDelay();
        LoadCell_2.tareNoDelay();
      }
      else if (secondByte == 'a') {
        LoadCell_0.tareNoDelay();
        LoadCell_1.tareNoDelay();
        LoadCell_2.tareNoDelay();
      }
    }
  } */
}  


void updateOLED(float totalTimeinSeconds, float weight0,float weight1,float weight2) { // Display elapsed time

  Serial.print(totalTimeinSeconds, 3);
  Serial.print(", ");
  displayWeight(weight0);
  Serial.print(weight0);
  Serial.print(", ");
  Serial.print(weight1);
  Serial.print(", ");
  Serial.println(weight2);

}

// Function to update OLED with weight and current time
void displayWeight(float weight) {
  display.clearDisplay(); // Clears display
  display.setTextColor(WHITE); // Text color is white

  // Weight title display
  display.setTextSize(1); 
  display.setCursor(0, 10); // Text will be drawn starting 10 pixels down from the top and at the far-left edge
  display.println("Weight:");
  
  // Weight display
  display.setTextSize(2); 
  display.setCursor(0, 30); // Text will be drawn starting 30 pixels down from the top and at the far-left edge
  display.print(weight, 2); // Print weight variable with 4 decimal places
  display.print(" g"); // Display grams (g) unit after weight variable

  display.display(); // Updates screen with new content
}