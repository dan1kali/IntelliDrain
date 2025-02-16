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
const byte redPin = 6, bluePin = 5, greenPin = 4, whitePin = 3;  // Pins for LEDs
const byte buttonPin = 2;  // Pin for button
const int command_out_pin = 30; //output to flushing Arduino

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
unsigned long blueLEDStartTime = 0;  // To track when blue LED is turned on
const long blueLEDTotalDuration = 10000;  // Total blue LED duration (10 seconds)
const long blueLEDOnDuration = 5000;  // Blue LED stays on for the first 5 seconds
const long blueLEDFlashDuration = 500;  // Flash interval (500ms)
bool blueLEDActive = false;  // Flag to check if blue LED is active
bool redLEDActive = false;  // Flag to check if red LED is active
int blueLEDOffCount = 0;  // Counter for how many times the blue LED has been turned off

///// THRESHOLD VARIABLE /////
float threshold = 0.0;  // Default threshold value
bool thresholdPromptDisplayed = false;  // Flag to check if the prompt has been displayed

///// BLUE LIGHT FLASHING /////
unsigned long flashInterval = 500;  // Flash interval in milliseconds
unsigned long lastFlashTime = 0;    // Store the time of the last flash
bool blueLEDState = false;          // Track whether blue LED is on or off

///// DISPLAY FUNCTION /////
// Function to update OLED with weight and current time
void displayWeightAndTime(float weight, unsigned long currentTime) {
  display.clearDisplay(); // Clears display
  display.setTextColor(WHITE); // Text color is white

  // Weight title display
  display.setTextSize(1); 
  display.setCursor(0, 10); // Text will be drawn starting 10 pixels down from the top and at the far-left edge
  display.println("Weight:");
  
  // Weight display
  display.setTextSize(2); 
  display.setCursor(0, 30); // Text will be drawn starting 30 pixels down from the top and at the far-left edge
  display.print(weight, 4); // Print weight variable with 4 decimal places
  display.print(" g"); // Display grams (g) unit after weight variable

  // Current Time display
  display.setTextSize(1);
  display.setCursor(0, 55); // Text will be drawn starting 55 pixels down from the top and at the far-left edge
  display.print("Time: ");
  display.print(currentTime); // Print currentTime variable
  display.print(" ms"); // Display grams (g) unit after weight variable

  display.display(); // Updates screen with new content
}

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
  float calibrationValue_0 = -408.7; // Calibration value for sensor load cell
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
    Serial.println("Timeout, check MCU>HX711 no.0 wiring and pin designations (occlusion sensor)");
  }
   if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations (weight scale 1)");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations (weight scale 2)");
  }

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
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(whitePin, OUTPUT);
  pinMode(buttonPin, INPUT); // Configure button pin as input with internal pull-up resistor
  // Start with only white LED on during baseline calculation
  digitalWrite(whitePin, HIGH);
  digitalWrite(redPin, LOW);  
  digitalWrite(bluePin, LOW);  
  digitalWrite(greenPin, LOW);  
  digitalWrite(command_out_pin, LOW);  
  Serial.println("System on!");
  
  ///// WAIT FOR THRESHOLD INPUT ///// 
  while (!Serial);  // Wait for the serial port to connect (necessary for some boards)
  
  Serial.println("Enter the threshold value:");
  // Wait for user input to set the threshold value
  while (Serial.available() == 0);  // Wait until data is available
  threshold = Serial.parseFloat();  // Read the threshold value from serial
  
  // Optionally, print the threshold to confirm it has been set
  Serial.print("Threshold set to: ");
  Serial.println(threshold);

  ///// LED CONTROL AFTER THRESHOLD SET ///// 
  digitalWrite(whitePin, LOW);  // Turn off white LED
  digitalWrite(greenPin, HIGH); // Turn on green LED to indicate threshold received
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
    digitalWrite(redPin, LOW);  // Turn off red LED
    redLEDActive = false;  // Reset the red LED active flag
    digitalWrite(greenPin, HIGH);  // Reactivate green LED
    blueLEDOffCount = 0;  // Reset the blue LED off counter
    Serial.println("Red LED turned off. Resume.");
    delay(200);  // Simple debounce delay
  }

  if (newDataReady) {
    if (millis() > t + updateInterval) {
      // Display elapsed time
      float totalTimeinSeconds = (currentMillis - startMillis) / 1000.0;
      Serial.print(totalTimeinSeconds, 3);
      Serial.print(", ");

      // Display current sensor weight values
      float weight0 = LoadCell_0.getData(); 
      displayWeightAndTime(weight0, currentMillis);
      Serial.print(weight0);
      Serial.print(", ");
      float weight1 = LoadCell_1.getData(); 
      float weight2 = LoadCell_2.getData(); 
      Serial.print(weight1);
      Serial.print(", ");
      Serial.println(weight2);

      if (redLEDActive) {
        digitalWrite(greenPin, LOW);  // Turn off green LED
        digitalWrite(bluePin, LOW);   // Turn off blue LED

            } else {
        // Check if the blue LED is not active (i.e., green LED should remain on)
        if (!blueLEDActive) {
          if (weight0 >= threshold) {
            if (!blueLEDActive) {  // Only activate if the blue LED is not already on
              digitalWrite(bluePin, HIGH);  // Turn on blue LED
              blueLEDStartTime = millis();  // Record the start time
              blueLEDActive = true;  // Set the flag to true to track the blue LED timing
              digitalWrite(greenPin, LOW);  // Turn off green LED when blue is activated
            }
          }
        }
      }

      // Logic for handling blue LED and red LED
      if (blueLEDActive) {
        // Calculate elapsed time since blue LED was turned on
        unsigned long elapsedTime = millis() - blueLEDStartTime;

        // First 5 seconds: Blue LED stays on
        if (elapsedTime < blueLEDOnDuration) {
          digitalWrite(bluePin, HIGH);  // Ensure the blue LED stays on
          digitalWrite(command_out_pin, HIGH); // Send command to start flush pump
          Serial.println("command out pin high");

        }
        // Last 5 seconds: Blue LED flashes
        else if (elapsedTime >= blueLEDOnDuration && elapsedTime < blueLEDTotalDuration) {
          // Flash blue LED every 500 milliseconds
          if (millis() - lastFlashTime >= flashInterval) {
            blueLEDState = !blueLEDState;  // Toggle LED state
            digitalWrite(bluePin, blueLEDState ? HIGH : LOW);  // Apply the new state
            digitalWrite(command_out_pin, HIGH); // Send command to start flush pump
            Serial.println("command out pin high (flashing)");
            lastFlashTime = millis();  // Update the last flash time
          }
        }

        // After 10 seconds, turn off the blue LED and restore green LED
        if (elapsedTime >= blueLEDTotalDuration) {
          digitalWrite(bluePin, LOW);  // Turn off blue LED
          digitalWrite(command_out_pin, LOW); // Send command to start flush pump
          Serial.println("command out pin low");
          blueLEDActive = false;  // Reset the flag
          digitalWrite(greenPin, HIGH);  // Turn green LED back on after blue LED is off
          blueLEDOffCount++;  // Increment the blue LED off counter

          // After the blue LED has been turned off twice, activate the red LED
          if (blueLEDOffCount >= 2 && !redLEDActive) {
            digitalWrite(redPin, HIGH);  // Turn on red LED
            redLEDActive = true;  // Set flag to track red LED status
            Serial.println("Red LED activated.");
          }
        }
      }

      // ** New logic for green LED to stay off when any other light is on ** 
      if (weight0 < threshold && !redLEDActive && !blueLEDActive) {
        digitalWrite(greenPin, HIGH); // Turn on green LED only if no other LED is active
        blueLEDOffCount = 0;  // Reset the blue LED off counter
      } else {
        digitalWrite(greenPin, LOW); // Turn off green LED otherwise
      }

      newDataReady = 0;
      t = millis();


    }
  }

  

  // receive command from serial terminal, send 't' to initiate tare operation:
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
  }
} 