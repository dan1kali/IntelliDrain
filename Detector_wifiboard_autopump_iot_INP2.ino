/* Comment this out to disable prints and save space */
//#define BLYNK_PRINT Serial

/* Fill in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID           "TMPL2GH6q6rHl"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "26WSWh6PdoD3rjZ6v-Y5cWXNVDI9qqN1"

#include <SPI.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>


///// LIBRARY INCLUSIONS /////
#include <HX711_ADC.h> // Communicates with HX711 load cell amplifier module


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Rice Visitor";
char pass[] = "";

///// PIN DEFINITIONS /////
const int LOADCELL_SCK_PIN = 7; // Sensor serial clock
const int LOADCELL_DOUT_PIN = 6; // Sensor load cell data
const byte redPin = 5, orangePin = 4, greenPin = 3;  // Pins for LEDs
const byte buttonResetPin = 1;  // Pin for reset button
const byte buttonFlushPin = 0; // Pin for flush button
const int command_out_pin = 2; //output to flushing

///// LOAD CELL SETUP /////
HX711_ADC LoadCell_0(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); // Sensor Load Cell

///// TIME VARIABLES /////
unsigned long t = 0;
unsigned long startMillis; // To store the start time
const int updateInterval = 954; //How often data is collected in milliseconds *********************************************

///// LOGIC AND LED CONTROL VARIABLES /////
unsigned long orangeLEDStartTime = 0;  // To track when orange LED is turned on
unsigned long twiceLEDStartTime = 0; // To track time between orange activations
unsigned long twiceLEDEndTime = 0; // To track time between orange activations
unsigned long timeBetween = 0; // To track time between orange activations
const long orangeLEDTotalDuration = 10000;  // Total orange LED duration (10 seconds)
const long orangeLEDOnDuration = 5000;  // orange LED stays on for the first 5 seconds
const long orangeLEDFlashDuration = 500;  // Flash interval (500ms)
const long TimeBetweenDuration = 5000; // Max unallowable time for orange activations
bool orangeLEDActive = false;  // Flag to check if orange LED is active
bool redLEDActive = false;  // Flag to check if red LED is active
bool OneCount = false; // Flag for first count of orange activation
bool OneCountDone = false;
bool TwoCount = false; // Flag for second count of orange activation
bool TwoCountDone = false;
int orangeLEDOffCount = 0;  // Counter for how many times the orange LED has been turned off
bool baselineNeeded = 1; 

///// THRESHOLD VARIABLE /////
float threshold = 0;  // Default threshold value

float openthreshold = 0;  // Default threshold value
float openthresholdsum = 0;  // Variable to store the sum of the readings
float openrecording; // Variable to store individual readings

float closedthreshold = 0;  // Default threshold value
float closedthresholdsum = 0;  // Variable to store the sum of the readings
float closedrecording; // Variable to store individual readings


///// orange LIGHT FLASHING SETTINGS /////
unsigned long flashInterval = 500;  // Flash interval in milliseconds
unsigned long lastFlashTime = 0;    // Store the time of the last flash
bool orangeLEDState = false;          // Track whether orange LED is on or off
bool greenLEDState = false;          // Track whether orange LED is on or off


///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// SETUP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void setup() {
  ///// SERIAL COMMUNICATION INITIALIZATION /////
  delay(3000);
  Serial.begin(9600);
  Serial.println("Starting...");


  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  ///// CALIBRATION VALUES /////
  float calibrationValue_0 = -286.04; // Calibration value for sensor load cell

  ///// LOAD CELL INITIALIZATION /////
  LoadCell_0.begin();

  ///// TARE & STABILIZATION /////
  unsigned long stabilizingtime = 2000; // tare precision can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // tare operation will be performed
  byte loadcell_0_rdy = 0;  
  LoadCell_0.start(stabilizingtime, _tare);
  delay(2000);


  ///// CURRENT TIME /////
  startMillis = millis(); // Stores time, in milliseconds, since the Arduino was powered on or reset (current time) ****************************

  if (LoadCell_0.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell_0.setCalFactor(calibrationValue_0); // set calibration value (float)
    Serial.println("Startup is complete");
  }


  ///// LED PIN INITIALIZATION /////
  pinMode(command_out_pin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(orangePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(buttonResetPin, INPUT_PULLUP); // Configure button pin as input with internal pull-up resistor
  pinMode(buttonFlushPin, INPUT_PULLUP); // Configure butto pin as input with internal pull-up resistor
  digitalWrite(redPin, HIGH);  
  digitalWrite(orangePin, HIGH);  
  digitalWrite(greenPin, LOW);  /// make it start green
  digitalWrite(command_out_pin, LOW);  
  Serial.println("System on!");
  
  delay(3000); 
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////// THRESHOLD CALCULATION ///////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  Serial.println("Starting auto-threshold calculation");

  unsigned long lastGreenLEDFlashTime = 0;    // Store the time of the last flash

  for (int i = 1; i <= 10; i++) {
    if (LoadCell_0.update()) {
      openrecording = LoadCell_0.getData(); // Take reading
      Serial.println(openrecording); // DELETE LATER
      openthresholdsum += openrecording; // Add current reading to sum
      delay(1000);
    } 
    
    /* if (millis() - lastGreenLEDFlashTime >= 500) {
      greenLEDState = !greenLEDState;  // Toggle the green LED state
      digitalWrite(greenPin, greenLEDState ? LOW : HIGH);  // Turn LED on or off based on the state
      lastGreenLEDFlashTime = millis();  // Update the last flash time
    } */

  }
  float openaverage = openthresholdsum / 10; // Calculate the average of the 10 readings
  Serial.println("Open average value: " + String(openaverage));

  Serial.println("Pinch tube");
  delay(5000);

  for (int i = 1; i <= 10; i++) {
    if (LoadCell_0.update()) {
      closedrecording = LoadCell_0.getData(); // Take reading
      Serial.println(closedrecording); // DELETE LATER
      closedthresholdsum += closedrecording; // Add current reading to sum
      delay(1000);
    } 

    /* if (millis() - lastGreenLEDFlashTime >= 500) {
      greenLEDState = !greenLEDState;  // Toggle the green LED state
      digitalWrite(greenPin, greenLEDState ? LOW : HIGH);  // Turn LED on or off based on the state
      lastGreenLEDFlashTime = millis();  // Update the last flash time
    } */

  }
  float closedaverage = closedthresholdsum / 10; // Calculate the average of the 10 readings
  Serial.println("Closed average value: " + String(closedaverage));

  threshold = 0.5 * (openaverage + closedaverage);
  Serial.println("Threshold average value: " + String(threshold));
  delay(5000);

  //threshold = -100;

 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ///// LED CONTROL AFTER THRESHOLD SET ///// 
  digitalWrite(greenPin, LOW); // Turn on green LED to indicate threshold received
  digitalWrite(orangePin, HIGH); // Turn off the other two LEDs
  digitalWrite(redPin, HIGH);

}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LOOP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void loop() {

  Blynk.run();

  ///// CURRENT TIME ///// 
  unsigned long currentMillis = millis();
  
  static boolean newDataReady = 0; 

  ///// DISPLAY CURRENT LOAD CELL DATA ///// 
  if (!redLEDActive) {  // Only update load cell readings if the red LED is not active
    if (LoadCell_0.update()) newDataReady = true;
  }

  // Check for button press to turn off red LED
  if (digitalRead(buttonResetPin) == LOW) {  // Button is pressed (assuming button is connected to ground)
    digitalWrite(redPin, HIGH);  // Turn off red LED
    redLEDActive = false;  // Reset the red LED active flag
    digitalWrite(greenPin, LOW);  // Reactivate green LED
    orangeLEDOffCount = 0;  // Reset the orange LED off counter
    OneCount = false;
    OneCountDone = false;
    TwoCount = false; // Flag for second count of orange activation
    TwoCountDone = false;

    // Serial.println("Pintout:" + string(digitalRead(buttonResetPin)));
    Serial.println("Red LED turned off. Resume.");
    delay(200);  // Simple debounce delay
  }

    // Check for button press to flush
  if (digitalRead(buttonFlushPin) == LOW) {  // Button is pressed (assuming button is connected to ground)
    orangeLEDActive = true; // Activate orange LED to begn flush
  }

  if (newDataReady) {
    if (millis() > t + updateInterval) {

      float totalTimeinSeconds = (currentMillis - startMillis) / 1000.0;
      float weight0 = LoadCell_0.getData(); 
      updateSerial(totalTimeinSeconds,weight0);

      if (redLEDActive) {
        digitalWrite(greenPin, HIGH);  // Turn off green LED
        digitalWrite(orangePin, HIGH);   // Turn off orange LED

        } 
        
        else {
        // Check if the orange LED is not active (i.e., green LED should remain on)
        if (!orangeLEDActive) {
          if (totalTimeinSeconds>=42) {
            if (weight0 <= threshold) {
              orangeLEDStartTime = millis();  // Record the start time
              orangeLEDActive = true;  // Set the flag to true to track the orange LED timing
              digitalWrite(greenPin, HIGH);  // Turn off green LED when orange is activated
            }
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
          digitalWrite(command_out_pin, LOW); // Send command to end flush pump
          Serial.println("command out pin low");
          orangeLEDActive = false;  // Reset the flag
          orangeLEDOffCount++;  // Increment the orange LED off counter
          Serial.println("orange LED count: " + String(orangeLEDOffCount));
          digitalWrite(greenPin, LOW);  // Turn green LED back on after orange LED is off
        }
      }

      if (orangeLEDOffCount == 1 && !OneCountDone) {
        twiceLEDStartTime = millis();
        Serial.println("orange start: " + String(twiceLEDStartTime));
        OneCount = true;
        OneCountDone = true;
      }

      if (orangeLEDOffCount == 2 && OneCount && !TwoCountDone) {
        twiceLEDEndTime = millis();
        Serial.println("orange end: " + String(twiceLEDEndTime));
        TwoCount = true;
        TwoCountDone = true;
      }

      if (TwoCount) {
        timeBetween = (twiceLEDEndTime - twiceLEDStartTime) - 10000;
        Serial.println("time between: " + String(timeBetween));
      }
          
        // After the orange LED has been turned off twice, activate the red LED
        if (timeBetween <= TimeBetweenDuration && TwoCount && !redLEDActive) {
         digitalWrite(redPin, LOW);  // Turn on red LED
         redLEDActive = true;  // Set flag to track red LED status
         Serial.println("Red LED activated.");
        }
            
        // if (timeBetween > TimeBetweenDuration && TwoCount) {
        //   orangeLEDOffCount = 0;
        // }
      

      // ** New logic for green LED to stay off when any other light is on ** 
      if (weight0 > threshold && !redLEDActive && !orangeLEDActive) {
        digitalWrite(greenPin, LOW); // Turn on green LED only if no other LED is active
      } else {
        digitalWrite(greenPin, HIGH); // Turn off green LED otherwise
      }

      newDataReady = 0;
      t = millis();
  
    }
  }




}  


void updateSerial(float totalTimeinSeconds, float weight0) { // Display elapsed time

  Serial.print(totalTimeinSeconds, 3);
  Serial.print(", ");
  Serial.println(weight0);
  tParam param;
  double int_totalTimeinSeconds = (double)totalTimeinSeconds;
  double int_weight0 = (double)weight0;
  Blynk.virtualWrite(V1, int_weight0);
  Blynk.virtualWrite(V2, int_totalTimeinSeconds);

}

