
///// LIBRARY INCLUSIONS /////
#include <HX711_ADC.h> // Communicates with HX711 load cell amplifier module
#include <SPI.h>

// Your WiFi credentials. Set password to "" for open networks.
char ssid[] = "Rice Visitor";
char pass[] = "";

///// NOVIRAD DEVICE PARAMETERS /////
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

// Flags and states
bool clog_yn = false, flush_now = false, red_led_stop = false;
bool RPwrstate = 0; //Assign variables
bool LPwrstate = 0;
int lastButtonState = -99;
int spdtstate = 0; // 1 = Normal, 0 = Off, -1 = prime
int flush_cycle_proceedyn = 0;

///// PIN DEFINITIONS /////
int in1 = 12, in2 = 13, in3 = 9, in4 = 10, pinSaline = 8, pinAbscess = 11;
int pinPwrA = A2, pinPwrB = A3;
int voltage_A_pin = A0;
const int LOADCELL_SCK_PIN = 3; // Sensor Serial clock
const int LOADCELL_DOUT_PIN = 4; // Sensor load cell data
const byte redPin = 14, orangePin = 15, greenPin = 16;  // Pins for LEDs
const byte buttonResetPin = 5;  // Pin for reset button
const byte buttonFlushPin = 6; // Pin for flush button

///// LOAD CELL SETUP /////
HX711_ADC LoadCell_0(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); // Sensor Load Cell

///// TIME VARIABLES /////
unsigned long t = 0; // To track time
unsigned long startMillis; // To store the start time
const int updateInterval = 954; // Data collection frequency in millis

///// LOGIC AND LED CONTROL VARIABLES /////
unsigned long orangeLEDStartTime = 0;  // To track when orange LED is turned on
int orangeLEDOffCount = 0;  // Counter for how many times the orange LED has been turned off
bool OneCount = false; // Flag for first count of orange activation
bool OneCountDone = false;
unsigned long twiceLEDStartTime = 0; // To track  start time between orange activations
bool TwoCount = false; // Flag for second count of orange activation
bool TwoCountDone = false;
unsigned long twiceLEDEndTime = 0; // To track end time between orange activations
unsigned long timeBetween = 0; // To calculate time between orange activations
const long orangeLEDTotalDuration = 10000;  // Total orange LED duration (10 seconds)
const long TimeBetweenDuration = 5000; // Max unallowable time for orange activations
bool orangeLEDActive = false;  // Flag to check if orange LED is active
bool redLEDActive = false;  // Flag to check if red LED is active

///// THRESHOLD VARIABLE /////
float threshold = 0;  // Default threshold value

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// SETUP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void setup() {
  ///// Serial COMMUNICATION INITIALIZATION /////
  delay(3000);
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
  pinMode(buttonResetPin, INPUT_PULLUP); // Configure button pin as input with internal pull-up resistor
  pinMode(buttonFlushPin, INPUT_PULLUP); // Configure butto pin as input with internal pull-up resistor
  digitalWrite(redPin, HIGH);  
  digitalWrite(orangePin, HIGH);  
  digitalWrite(greenPin, LOW);  /// Start with green LED
  Serial.println("System on!");
  
  delay(3000); 

}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LOOP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void loop() {
  static boolean newDataReady = false;
  static byte thresholdStep = 0;
  static byte openIndex = 0, closedIndex = 0;
  static float openReadings[10], closedReadings[10];
  static bool thresholdCalculated = false;
  static bool case2Prompted = false;

  static bool greenLEDState = false;
  static unsigned long lastGreenBlink = 0;
  static unsigned long lastOpenReadTime = 0;
  static unsigned long lastClosedReadTime = 0;

  unsigned long currentMillis = millis();

  ///// -------------------- THRESHOLD CALCULATION PHASE -------------------- /////
  if (!thresholdCalculated) {
    switch (thresholdStep) {
      case 0: { // Collect open readings
        // Blink green LED slowly (1 sec)
        if (currentMillis - lastGreenBlink >= 1000) {
          greenLEDState = !greenLEDState;
          digitalWrite(greenPin, greenLEDState ? LOW : HIGH);
          lastGreenBlink = currentMillis;
        }

        // Take one reading every 1000ms
        if (currentMillis - lastOpenReadTime >= 1000 && LoadCell_0.update()) {
          openReadings[openIndex] = LoadCell_0.getData();
          Serial.println("Open reading " + String(openIndex + 1) + ": " + String(openReadings[openIndex]));
          openIndex++;
          lastOpenReadTime = currentMillis;
        }

        if (openIndex >= 10) {
          thresholdStep = 2;
        }
        return;
      }

      case 2: { // Prompt and collect closed readings
        if (!case2Prompted) {
          Serial.println("Please pinch the tube...");
          case2Prompted = true;
          delay(3000);  // Give user time to pinch the tube
          Serial.println("Collecting closed readings...");
        }

        // Blink green LED rapidly (200ms)
        if (currentMillis - lastGreenBlink >= 200) {
          greenLEDState = !greenLEDState;
          digitalWrite(greenPin, greenLEDState ? LOW : HIGH);
          lastGreenBlink = currentMillis;
        }

        // Take one reading every 1000ms
        if (currentMillis - lastClosedReadTime >= 1000 && LoadCell_0.update()) {
          closedReadings[closedIndex] = LoadCell_0.getData();
          Serial.println("Closed reading " + String(closedIndex + 1) + ": " + String(closedReadings[closedIndex]));
          closedIndex++;
          lastClosedReadTime = currentMillis;
        }

        if (closedIndex >= 10) {
          float openSum = 0, closedSum = 0;
          for (int i = 0; i < 10; i++) {
            openSum += openReadings[i];
            closedSum += closedReadings[i];
          }

          float openAvg = openSum / 10.0;
          float closedAvg = closedSum / 10.0;
          threshold = 0.5 * (openAvg + closedAvg);

          Serial.println("Open avg: " + String(openAvg));
          Serial.println("Closed avg: " + String(closedAvg));
          Serial.println("Threshold set to: " + String(threshold));

          thresholdCalculated = true;
          digitalWrite(greenPin, LOW); // turn ON steady
        }
        return;
      }
    }
  }

  ///// -------------------- OPERATIONAL PHASE -------------------- /////

  if (!redLEDActive && LoadCell_0.update()) {
    newDataReady = true;
  }

  // Reset red LED
  if (digitalRead(buttonResetPin) == LOW) {
    digitalWrite(redPin, HIGH);
    redLEDActive = false;
    red_led_stop = false;
    digitalWrite(greenPin, LOW);
    orangeLEDOffCount = 0;
    OneCount = false;
    OneCountDone = false;
    TwoCount = false;
    TwoCountDone = false;
    Serial.println("Red LED turned off. Resume.");
    delay(200);
  }

  if (newDataReady && currentMillis > t + updateInterval && !flush_now) {
    float totalTimeinSeconds = (currentMillis - startMillis) / 1000.0;
    float weight0 = LoadCell_0.getData();
    updateSerial(totalTimeinSeconds, weight0);

    // LED logic
    if (redLEDActive) {
      digitalWrite(greenPin, HIGH);
      digitalWrite(orangePin, HIGH);
    } else {
      if (!orangeLEDActive && totalTimeinSeconds >= 42 && weight0 <= threshold) {
        orangeLEDStartTime = millis();
        orangeLEDActive = true;
        digitalWrite(greenPin, HIGH);
      }
    }

    // Manual flush button press
    if (digitalRead(buttonFlushPin) == LOW && !orangeLEDActive) {
      Serial.println("Manual flush triggered.");
      orangeLEDStartTime = millis();
      orangeLEDActive = true;
      digitalWrite(greenPin, HIGH);
      flush_now = true;
      delay(200); // debounce
    }

    // Orange LED operation
    if (orangeLEDActive) {
      unsigned long elapsedTime = millis() - orangeLEDStartTime;

      if (elapsedTime < orangeLEDTotalDuration) {
        digitalWrite(orangePin, LOW);
        clog_yn = true; 
        Serial.println("clog_yn set to true");
      } 

      if (elapsedTime >= orangeLEDTotalDuration) {
        digitalWrite(orangePin, HIGH);
        clog_yn = false;
        orangeLEDActive = false;
        orangeLEDOffCount++;
        Serial.println("orange LED count: " + String(orangeLEDOffCount));
        digitalWrite(greenPin, LOW);
      }
    }

    // Orange LED tracking logic
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
      timeBetween = (twiceLEDEndTime - twiceLEDStartTime) - orangeLEDTotalDuration;
      Serial.println("time between: " + String(timeBetween));
    }

    // Red LED logic
    if (TwoCount && !redLEDActive) {
      if (timeBetween <= TimeBetweenDuration) {
        digitalWrite(redPin, LOW);
        redLEDActive = true;
        red_led_stop = true;
        Serial.println("Red LED activated.");
      } else {
        orangeLEDOffCount = 0;
        OneCount = false;
        OneCountDone = false;
        TwoCount = false;
        TwoCountDone = false;
        Serial.println("No problems. Continue detection.");
      }
    }

    // Green LED default (on when idle)
    if (weight0 > threshold && !redLEDActive && !orangeLEDActive) {
      digitalWrite(greenPin, LOW);
    } else {
      digitalWrite(greenPin, HIGH);
    }

    newDataReady = false;
    t = currentMillis;
  }

  ////////////////////////////////////////////////////////////
  ///////////// NOVIRAD MAIN LOOP FUNCTION ///////////////////
  ////////////////////////////////////////////////////////////

  // Timer-based pump control for periodic flushing
  flush_cycle_proceedyn = handle_flush_cycle();
  if (flush_cycle_proceedyn == 0) {return;}

  // CHECK FOR CLOG FUNCTION, then turn clog_yn flag to true  //

  // Handle pump logic based on sensor readings and states
  handle_pump_logic();

}

void updateSerial(float totalTimeinSeconds, float weight0) { // Display elapsed time

  Serial.print(totalTimeinSeconds, 3);
  Serial.print(", ");
  Serial.println(weight0);

}

////////////////////////////////////////////////////////////
///////////////// HANDLE TOGGLE SWITCH /////////////////////
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
///////////////// PUMP LOGIC FUNCTION //////////////////////
////////////////////////////////////////////////////////////

void handle_pump_logic() {

  ////////////////////////////////////////
  // Normal operation (no clog, no flush)
  if (!flush_now && !clog_yn && !red_led_stop) {
    startAbscessSuction();
    stopSaline();
  }

  else if (red_led_stop) {
  stopAbscess();
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
////////////// FLUSH CYCLE START FUNCTION //////////////////
////////////////////////////////////////////////////////////

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
///////////////// TIMER FUNCTION ///////////////////////////
////////////////////////////////////////////////////////////

void setup_timer() {
  flush_timer_start = millis();

}

////////////////////////////////////////////////////////////
////////////// UPDATE SETTINGS FUNCTION ////////////////////
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
//////////////// PUMP CONTROL FUNCTION /////////////////////
////////////////////////////////////////////////////////////

void update_pump_rpm() {
  analogWrite(pinSaline, saline_rpm);
  analogWrite(pinAbscess, abscess_rpm);
}

////////////////////////////////////////////////////////////
////////////////// PUMP CONTROL HELPERS ////////////////////
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

