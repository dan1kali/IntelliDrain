/* Comment this out to disable prints and save space */
//#define BLYNK_PRINT Serial

/* Fill in information from Blynk Device Info here */
/* #define BLYNK_TEMPLATE_ID           "TMPL2GH6q6rHl"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "26WSWh6PdoD3rjZ6v-Y5cWXNVDI9qqN1" */

#include <SPI.h>
// #include <WiFiNINA.h>
// #include <BlynkSimpleWiFiNINA.h>
#include <HX711_ADC.h> // Communicates with HX711 load cell amplifier module

/////////////////////From NoviRad Code////////////////////
/* #include "sec_func.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h" */
////////////////////////////////////////////////////////////

///////////////Novirad Device parameters
int saline_rpm = 220; //Saline pump rpm: scaled 0 -255
int abscess_rpm = 170;   //Saline pump rpm: scaled 0 -255
int flush_duration = 10; //Saline flush duration: seconds
float flush_frequency = 100; //Saline flush frequency: minutes
bool verbosex = true;

//Tracking metrics
float totalflushvol = 0.0, totalflushtime = 0.0;

// Timers
unsigned long absess_flush_timer_start = 0; //Flush performed when clog is detected
unsigned long absess_flush_timer_limit = 0;
unsigned long flush_timer_start = 0; //Flush performed when periodic flush is performed
unsigned long flush_timer_limit = 0;
bool timer_enabled = true;

// Flags and states
bool clog_yn = false, flush_now = false;
int count = 0;
bool RPwrstate = 0; //Assign variables
bool LPwrstate = 0;
int lastButtonState = -99;
int spdtstate = 0; // 1 = Normal, 0 = Off, -1 = prime
int spdt_proceedyn = 0;
int flush_cycle_proceedyn = 0;

// Pin assignments
int in1 = 10, in2 = 11, in3 = 4, in4 = 5, pinSaline = 3,pinAbscess = 9;
int pinPwrA = A2, pinPwrB = A3;
int voltage_A_pin = A0;

////////////////////////////////////////////////////////////

///// PIN DEFINITIONS /////
const int LOADCELL_SCK_PIN = 42; // Sensor serial clock
const int LOADCELL_DOUT_PIN = 40; // Sensor load cell data
const byte redPin = 16, orangePin = 15, greenPin = 14;  // Pins for LEDs
const byte buttonPin = 6;  // Pin for button

// Your WiFi credentials, set password to "" for open networks.
char ssid[] = "Rice Visitor";
char pass[] = "";

///// LOAD CELL SETUP /////
HX711_ADC LoadCell_0(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); // Sensor Load Cell

///// TIME VARIABLES /////
unsigned long t = 0;
unsigned long startMillis; // To store the start time
const int updateInterval = 954; //How often data is collected in milliseconds *********************************************

///// LOGIC AND LED CONTROL VARIABLES /////
unsigned long orangeLEDStartTime = 0;  // To track when orange LED is turned on
const long orangeLEDTotalDuration = 9500;  // Total orange LED duration (10 seconds)
const long orangeLEDOnDuration = 10000;  // orange LED stays on for the first 5 seconds
const long orangeLEDFlashDuration = 500;  // Flash interval (500ms)
bool orangeLEDActive = false;  // Flag to check if orange LED is active
bool redLEDActive = false;  // Flag to check if red LED is active
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
  delay(3000);

  ////////////////////////////////////////////////////////////
  // SETUP FUNCTION
  ////////////////////////////////////////////////////////////
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Starting...");

  // Initialize pump control pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(pinSaline, OUTPUT);
  pinMode(pinAbscess, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinPwrA, INPUT_PULLUP); //SPDT
  pinMode(pinPwrB, INPUT_PULLUP); //SPDT

  //Update pumps rpm
  update_pump_rpm();
  
  // Initialize timer and BLE setup
  setup_timer();
  Serial.println("Pump Control System Ready");

  // Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("IoT System Ready");

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
  pinMode(redPin, OUTPUT);
  pinMode(orangePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Configure button pin as input with internal pull-up resistor

  
  digitalWrite(redPin, HIGH);  
  digitalWrite(orangePin, HIGH);  
  digitalWrite(greenPin, LOW);  /// make it start green
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
    
    if (millis() - lastGreenLEDFlashTime >= 500) {
      greenLEDState = !greenLEDState;  // Toggle the green LED state
      digitalWrite(greenPin, greenLEDState ? LOW : HIGH);  // Turn LED on or off based on the state
      lastGreenLEDFlashTime = millis();  // Update the last flash time
    }

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

    if (millis() - lastGreenLEDFlashTime >= 500) {
      greenLEDState = !greenLEDState;  // Toggle the green LED state
      digitalWrite(greenPin, greenLEDState ? LOW : HIGH);  // Turn LED on or off based on the state
      lastGreenLEDFlashTime = millis();  // Update the last flash time
    }

  }
  float closedaverage = closedthresholdsum / 10; // Calculate the average of the 10 readings
  Serial.println("Closed average value: " + String(closedaverage));

  threshold = 0.5 * (openaverage + closedaverage);
  Serial.println("Threshold average value: " + String(threshold));
  delay(5000);

  //threshold = -100;

  ///// LED CONTROL AFTER THRESHOLD SET ///// 
  digitalWrite(greenPin, LOW); // Turn on green LED to indicate threshold received
  digitalWrite(orangePin, HIGH); // Turn off the other two LEDs
  digitalWrite(redPin, HIGH);

}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LOOP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
int orangecounter = 0;

void loop() {

  //Blynk.run();

  ///// CURRENT TIME ///// 
  unsigned long currentMillis = millis();
  static boolean newDataReady = 0; 

  ///// DISPLAY CURRENT LOAD CELL DATA ///// 
  if (!redLEDActive) {  // Only update load cell readings if the red LED is not active
    if (LoadCell_0.update()) newDataReady = true;
  }

  // Check for button press to turn off red LED
  if (digitalRead(buttonPin) == LOW) {  // Button is pressed (assuming button is connected to ground)
    digitalWrite(redPin, HIGH);  // Turn off red LED
    redLEDActive = false;  // Reset the red LED active flag
    digitalWrite(greenPin, LOW);  // Reactivate green LED
    orangeLEDOffCount = 0;  // Reset the orange LED off counter
    // Serial.println("Pintout:" + string(digitalRead(buttonPin)));
    Serial.println("Red LED turned off. Resume.");
    delay(200);  // Simple debounce delay
  }

  if (newDataReady) {
    if (millis() > t + updateInterval) {

      float totalTimeinSeconds = (currentMillis - startMillis) / 1000.0;
      float weight0 = LoadCell_0.getData(); 
      updateSerial(totalTimeinSeconds,weight0);

      if (redLEDActive) {
        digitalWrite(greenPin, HIGH);  // Turn off green LED
        digitalWrite(orangePin, HIGH);   // Turn off orange LED
        //digitalWrite(command_stop_pin, HIGH);  /////////////////// new
        } 
        
        else {

        //digitalWrite(command_stop_pin, LOW); /////////////////////// new
        
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

        // First 10 seconds: orange LED stays on
        if (elapsedTime < orangeLEDOnDuration) {
          digitalWrite(orangePin, LOW);  // Ensure the orange LED stays on
          
          // digitalWrite(command_out_pin, HIGH); // Send command to start flush pump
          clog_yn = true; // replace with this flag
          Serial.println("clog_yn set to true");

        }

        // After 10 seconds, turn off the orange LED and restore green LED
        if (elapsedTime >= orangeLEDTotalDuration) {
          digitalWrite(orangePin, HIGH);  // Turn off orange LED
          orangecounter++;
          
          //digitalWrite(command_out_pin, LOW); // Send command to start flush pump
          clog_yn = false; // replace with this flag
          Serial.println("clog_yn set to false");

          orangeLEDActive = false;  // Reset the flag
          digitalWrite(greenPin, LOW);  // Turn green LED back on after orange LED is off
          orangeLEDOffCount++;  // Increment the orange LED off counter

          // After the orange LED has been turned off twice, activate the red LED
        }
      }

      
/*       // Logic for handling orange LED and red LED
      if (orangeLEDActive) {
        // Calculate elapsed time since orange LED was turned on
        unsigned long elapsedTime = millis() - orangeLEDStartTime;

        // First 5 seconds: orange LED stays on
        if (elapsedTime < orangeLEDOnDuration) {
          digitalWrite(orangePin, LOW);  // Ensure the orange LED stays on
          
          // digitalWrite(command_out_pin, HIGH); // Send command to start flush pump
          clog_yn = true; // replace with this flag
          Serial.println("command out pin high");

        }
        // Last 5 seconds: orange LED flashes
        else if (elapsedTime >= orangeLEDOnDuration && elapsedTime < orangeLEDTotalDuration) {
          // Flash orange LED every 500 milliseconds
          if (millis() - lastFlashTime >= flashInterval) {
            orangeLEDState = !orangeLEDState;  // Toggle LED state
            digitalWrite(orangePin, orangeLEDState ? LOW : HIGH);  // Apply the new state
            
            
            // digitalWrite(command_out_pin, HIGH); // Send command to start flush pump
            clog_yn = true; // replace with this flag
            Serial.println("command out pin high (flashing)");
            
            lastFlashTime = millis();  // Update the last flash time
          }
        }

        // After 10 seconds, turn off the orange LED and restore green LED
        if (elapsedTime >= orangeLEDTotalDuration) {
          digitalWrite(orangePin, HIGH);  // Turn off orange LED
          orangecounter++;
          
          
          //digitalWrite(command_out_pin, LOW); // Send command to start flush pump
          clog_yn = false; // replace with this flag
          Serial.println("command out pin low");


          orangeLEDActive = false;  // Reset the flag
          digitalWrite(greenPin, LOW);  // Turn green LED back on after orange LED is off
          orangeLEDOffCount++;  // Increment the orange LED off counter

          // After the orange LED has been turned off twice, activate the red LED
          if (orangecounter == 4) {
            digitalWrite(redPin, LOW);  // Turn on red LED
            redLEDActive = true;  // Set flag to track red LED status
            Serial.println("Red LED activated.");
          }
        }
      } */

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


  ////////////////////////////////////////////////////////////
  // NOVIRAD MAIN LOOP FUNCTION
  ////////////////////////////////////////////////////////////

  //Check toggle switch status (e.g., on, prime, off) 
  //spdt_proceedyn = handle_toggle_switch();
  //if (spdt_proceedyn == 0) {return;}

  
  // Timer-based pump control for periodic flushing
  flush_cycle_proceedyn = handle_flush_cycle();
  if (flush_cycle_proceedyn == 0) {return;}

  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  // CHECK FOR CLOG FUNCTION, then turn clog_yn flag to true  //
  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////

  // Handle pump logic based on sensor readings and states
  handle_pump_logic();

}  


void updateSerial(float totalTimeinSeconds, float weight0) { // Display elapsed time

  Serial.print(totalTimeinSeconds, 3);
  Serial.print(", ");
  Serial.println(weight0);
  /* tParam param;
  double int_totalTimeinSeconds = (double)totalTimeinSeconds;
  double int_weight0 = (double)weight0;
  Blynk.virtualWrite(V1, int_weight0);
  Blynk.virtualWrite(V2, int_totalTimeinSeconds); */
}



////////////////////////////////////////////////////////////
// HANDLE TOGGLE SWITCH
////////////////////////////////////////////////////////////
int handle_toggle_switch(){

    RPwrstate = digitalRead(pinPwrA);  //read the input pins
    LPwrstate = digitalRead(pinPwrB);
  
    if ( (RPwrstate == LOW) && (LPwrstate == HIGH)) //test for right position
    {
      spdtstate = 1; // 1 = Normal, 0 = Off, -1 = prime
      //Serial.println("Switch in Right position");
    }
    if ( (RPwrstate == HIGH) && (LPwrstate == HIGH)) //test for center position
    {
      spdtstate = 0; // 1 = Normal, 0 = Off, -1 = prime    
      //Serial.println("Switch in Center position");
    }
    if ( (RPwrstate == HIGH) && (LPwrstate == LOW))  //test for left position
    {
      spdtstate = -1; // 1 = Normal, 0 = Off, -1 = prime    
      //Serial.println("Switch in Left position");
    } 
  
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    //Once device is powered on, get current position of the switch and create last button state (runs once)
    if (lastButtonState == -99){  
      lastButtonState = spdtstate;
    }

    //Check for a change in toggle switch state
    if (spdtstate != lastButtonState && spdtstate == 1) { //Switched from off or prime to on  
        //Serial.println("Switched from prime/off position to on position");
        setup_timer();  
      }
    else if (spdtstate != lastButtonState && spdtstate == 0) { //Switched from on or prime to off
        //Serial.println("Switched from on/prime position to off position");
        spdtstate = 0;
    }

    //Update spdtstate
    lastButtonState = spdtstate;

    //Change pump settings depending on toggle switch state
    if (spdtstate == 0){ //Off position
      stopAbscess();
      stopSaline();
      return 0; //Flag to repeat main loop
    }
   else if (spdtstate == -1){ //Primes position
      stopAbscess();
      startSalinePropulsion();
      return 0; //Flag to repeat main loop
    }
   else if (spdtstate == 1){ //On position
      return 1; //Flag to proceed to remaining code
   }
  
}



////////////////////////////////////////////////////////////
// PUMP LOGIC FUNCTION
////////////////////////////////////////////////////////////

void handle_pump_logic() {

  ////////////////////////////////////////
  // Normal operation (no clog, no flush)
  if (!flush_now && !clog_yn) {
    startAbscessSuction();
    stopSaline();
  }

  ////////////////////////////////////////
  // Clogged operation (reverse flow and flush)
  else if (clog_yn || flush_now) {

   //Stop pumps for a 1/2 second
   stopAbscess();
   stopSaline();
   //delay(500);    
    
    // Reverse pump and start saline propulsion for flushing
    startAbscessReverse();
    startSalinePropulsion();

    // Initialize clog clear timer (abscess_flush_timer)
    absess_flush_timer_start = millis();
    absess_flush_timer_limit = flush_duration * 1000; // Wait for stabilization

    Serial.print("CLOG FLUSHING FOR: ");
    Serial.print(flush_duration);
    Serial.println(" seconds");
    while (millis() - absess_flush_timer_start < absess_flush_timer_limit) {
      //Serial.println("Reversing pump and flush to clear clog");
    } 
      // Flush completed
      flush_now = false;
      clog_yn = false;
      totalflushtime += flush_duration; //Log flush duration
      Serial.println("CLOG FLUSHING COMPLETED!");

      //RESET PERIODIC TIMER
      setup_timer();

      //Stop pumps for a 1/2 second
      stopAbscess();
      stopSaline();
      //delay(500);     
  }
}

////////////////////////////////////////////////////////////
// FLUSH CYCLE START FUNCTION
////////////////////////////////////////////////////////////

/*

// changed millis() to current time, added first Serial.printIn
// --> only brief hiccup after 60s, no flush. Still errors.

int handle_flush_cycle() {
  unsigned long current_time = millis();
 
  Serial.println(current_time - flush_timer_start);
  if (current_time - flush_timer_start >= flush_frequency*60.0*1000.0) {   
      // Serial.println("Starting flush cycle"); 
      stopAbscess();
      startSalinePropulsion();
      if (current_time - flush_timer_start > flush_frequency*60.0*1000.0 + flush_duration*1000.0){ //After flush duration is completed, restore normal function
        //Flush cycle completed
        setup_timer(); //Reset time
        totalflushtime += flush_duration; //Log flush duration
      return 1;
      }
      else { //Flush is not completed. Repeat loop
      return 0;
      }
  }
  return 1;
}

*/


// working version, no one knows why
int handle_flush_cycle() { 
  unsigned long current_time = millis();
  //Serial.print("Time elapsed since last flush: ");
  //Serial.println(current_time - flush_timer_start);
  
  if (current_time - flush_timer_start >= flush_frequency * 60.0 * 1000.0) {   
    Serial.println("Starting periodic flush cycle...");
    // Start the flush cycle (stop abscess, start saline)
    stopAbscess();
    startSalinePropulsion();
    
    // Wait for the flush duration to complete
    if (current_time - flush_timer_start >= flush_frequency * 60.0 * 1000.0 + flush_duration * 1000.0) {
      // Reset timer after flush cycle
      setup_timer();
      totalflushtime += flush_duration; // Log flush duration
      Serial.println("Flush cycle completed.");
      return 1;
    } else {
      // Flush not yet completed
      return 0;
    }
  }
  return 1; // Default return value if time condition is not met
}



////////////////////////////////////////////////////////////
// TIMER FUNCTION
////////////////////////////////////////////////////////////

void setup_timer() {
  flush_timer_start = millis();

}

////////////////////////////////////////////////////////////
// UPDATE SETTINGS FUNCTION
////////////////////////////////////////////////////////////

void update_settings(String abscess_power, String flush_power, String flush_volume, String flush_freq, String auto_flush) {
  // Convert BLE data into pump parameter values
  abscess_rpm = round(abscess_power.toInt() / 1000.0 * 255);
  saline_rpm = round(flush_power.toInt() / 1000.0 * 255);
  flush_duration = round(flush_volume.toInt() * 2 * (100 / (float)saline_rpm));
  flush_frequency = flush_freq.toInt();
  
  if (auto_flush.toInt() == 0) {
     //TODO: auto-flushing turned off
    }
  else if (auto_flush.toInt() == 1){
    //TODO: auto-flushing turned on      
  }
  else if (auto_flush.toInt() == 2){
    Serial.println("Prompted immediate flush of the system");
    flush_now = true;    
  }

  update_pump_rpm();
  setup_timer();
}

////////////////////////////////////////////////////////////
// PUMP CONTROL FUNCTION
////////////////////////////////////////////////////////////

void update_pump_rpm() {
  analogWrite(pinSaline, saline_rpm);
  analogWrite(pinAbscess, abscess_rpm);
}

////////////////////////////////////////////////////////////
// PUMP CONTROL HELPERS
////////////////////////////////////////////////////////////

void stopAbscess() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void stopSaline() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void startAbscessSuction() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void startAbscessReverse() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void startSalinePropulsion() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}


void printVerbose(const String& message) {
    if (verbosex) {
        Serial.println(message);
    }
}

