
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
float totalflushtime = 0.0;
int flushcounter = 0, prevflushcounter = 0, didaflushtrigger = 0;

// Timers
unsigned long abscess_flush_timer_start = 0; //Flush performed when clog is detected
unsigned long abscess_flush_timer_limit = 0;
unsigned long flush_timer_start = 0; //Flush performed when periodic flush is performed

// Flags and states
bool clog_yn = false, flush_now = false, red_led_stop = false;

///// PIN DEFINITIONS /////
int in1 = 12, in2 = 11, in3 = 9, in4 = 10, pinSaline = 8, pinAbscess = 13;
const int sensorSckPin = 3; // Load cell clock
const int sensorDoutPin = 4; // Load cell data
const int drainageSckPin = 18; // Drainage weight scale clock
const int drainageDoutPin = 17; // Drainage weight scale data
const int salineSckPin = 20; // Saline weight scale clock
const int salineDoutPin = 19; // Saline weight scale data
const byte redPin = 14, orangePin = 15, greenPin = 16;  // Pins for LEDs
const byte buttonResetPin = 5;  // Pin for reset button
const byte buttonFlushPin = 6; // Pin for flush button

///// LOAD CELL SETUP /////
HX711_ADC Sensor_LoadCell(sensorDoutPin, sensorSckPin); // Sensor Load Cell
HX711_ADC Drainage_WeightScale(drainageDoutPin, drainageSckPin); // Sensor Load Cell
HX711_ADC Saline_WeightScale(salineDoutPin, salineSckPin); // Sensor Load Cell

///// TIME VARIABLES /////
unsigned long t = 0; // To track time
unsigned long startMillis; // To store the start time
const int updateInterval = 1000; // Data collection frequency in millis

///// LOGIC AND LED CONTROL VARIABLES /////
unsigned long orangeLEDStartTime = 0;  // To track when orange LED is turned on
unsigned long twiceLEDStartTime = 0; // To track time between orange activations
unsigned long twiceLEDEndTime = 0; // To track time between orange activations
unsigned long timeBetween = 0; // To track time between orange activations
const long orangeLEDTotalDuration = 10000;  // Total orange LED duration (10 seconds)
const long TimeBetweenDuration = 5000; // Max unallowable time for orange activations
bool orangeLEDActive = false;  // Flag to check if orange LED is active
bool redLEDActive = false;  // Flag to check if red LED is active
bool OneCount = false; // Flag for first count of orange activation
bool OneCountDone = false;
bool TwoCount = false; // Flag for second count of orange activation
bool TwoCountDone = false;
int orangeLEDOffCount = 0;  // Counter for how many times the orange LED has been turned off
bool baselineNeeded = 1; 
bool ResetHandled = false;

///// THRESHOLD VARIABLE /////
float threshold = 0;  // Default threshold value

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// SETUP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void setup() {
  ///// Serial COMMUNICATION INITIALIZATION /////
  delay(3000);
  Serial.begin(9600);


  ///// Initialize buttons, press reset button to start program /////
  pinMode(buttonResetPin, INPUT_PULLUP); // Configure button pin as input with internal pull-up resistor
  pinMode(buttonFlushPin, INPUT_PULLUP); // Configure butto pin as input with internal pull-up resistor
  Serial.println("push power button, then reset button to start");
  while (digitalRead(buttonResetPin) == HIGH){
    // wait for reset button to be pushed
  }

  // Initialize pump control pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(pinSaline, OUTPUT);
  pinMode(pinAbscess, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //Update pumps rpm
  update_pump_rpm();
  
  // Initialize timer and BLE setup
  setup_timer();
  Serial.println("Pump Control System Ready");

  ///// CALIBRATION VALUES /////
  float calibrationValue_Sensor_LoadCell = -286.04; // Calibration value for sensor load cell
  float calibrationValue_Drainage_WeightScale = 230; // Calibration value for drainage weight scale
  float calibrationValue_Saline_WeightScale = 230; // Calibration value for saline weight scale

  ///// LOAD CELL INITIALIZATION /////
  Sensor_LoadCell.begin();
  Drainage_WeightScale.begin();
  Saline_WeightScale.begin();

  ///// TARE & STABILIZATION OF WEIGHT SCALES /////
  unsigned long stabilizingtime = 2000; // tare precision can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // tare operation will be performed
  Drainage_WeightScale.start(stabilizingtime, _tare);
  Saline_WeightScale.start(stabilizingtime, _tare);

  ///// CURRENT TIME /////
  startMillis = millis(); // Stores time, in milliseconds, since the Arduino was powered on or reset (current time) ****************************

  ///// START PUMPS /////
  startAbscessSuction();
  delay(5000); // Wait 5 seconds for flow to stabilize before tare

  ///// TARE & STABILIZATION OF OCCLUSION SENSOR /////
  Sensor_LoadCell.start(stabilizingtime, _tare);

  if (Sensor_LoadCell.getTareTimeoutFlag() || Drainage_WeightScale.getTareTimeoutFlag() || Saline_WeightScale.getTareTimeoutFlag()) {
    Serial.println("Timeout, check load cell and weight scale wiring and pin designations");
    while (1);
  }
  else {
    Sensor_LoadCell.setCalFactor(calibrationValue_Sensor_LoadCell); // set calibration value (float)
    Drainage_WeightScale.setCalFactor(calibrationValue_Drainage_WeightScale);
    Saline_WeightScale.setCalFactor(calibrationValue_Saline_WeightScale);
    Serial.println("Startup is complete");
  }

  ///// LED PIN INITIALIZATION /////
  pinMode(redPin, OUTPUT);
  pinMode(orangePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
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
  unsigned long currentMillis = millis();
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

  static bool sensorStabilized = false;
  static int stabilizationCounter = 0;

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
        if (currentMillis - lastOpenReadTime >= 1000 && Sensor_LoadCell.update()) {
          openReadings[openIndex] = Sensor_LoadCell.getData();
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
        if (currentMillis - lastClosedReadTime >= 1000 && Sensor_LoadCell.update()) {
          closedReadings[closedIndex] = Sensor_LoadCell.getData();
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
          sensorStabilized = false;
          stabilizationCounter = 0;
        }
        return;
      }
    }
  }

  ///// -------------------- OPERATIONAL PHASE -------------------- /////

  ///// DISPLAY CURRENT LOAD CELL DATA ///// 
  if (!redLEDActive) {
    if (Sensor_LoadCell.update() && Drainage_WeightScale.update() && Saline_WeightScale.update()) newDataReady = true;
  }

  // Check for button press to turn off red LED
  if (digitalRead(buttonResetPin) == LOW && !ResetHandled) {  // Button is pressed (assuming button is connected to ground)
    digitalWrite(redPin, HIGH);  // Turn off red LED
    redLEDActive = false;  // Reset the red LED active flag
    digitalWrite(greenPin, LOW);  // Reactivate green LED
    orangeLEDOffCount = 0;  // Reset the orange LED off counter
    OneCount = false;
    OneCountDone = false;
    TwoCount = false;
    TwoCountDone = false;
    ResetHandled = true;
    delay(200);  // Simple debounce delay
  }

  if (digitalRead(buttonResetPin) == HIGH) {
    ResetHandled = false;  // Allow next press to be registered
  }

  // Manual flush button press
  if (digitalRead(buttonFlushPin) == LOW && !orangeLEDActive) {
   //Serial.println("Manual flush triggered.");
    orangeLEDStartTime = millis();
    orangeLEDActive = true;
    digitalWrite(greenPin, HIGH);
    flush_now = true;
    delay(200); // debounce
  }

  if (newDataReady && millis() > (t + updateInterval) && !flush_now) {

    float totalTimeinSeconds = (currentMillis - startMillis) / 1000.0;
    
    float occlusionSensorValue = Sensor_LoadCell.getData();
    float drainageVolume = Drainage_WeightScale.getData();
    float salineVolume = Saline_WeightScale.getData();

      if (flushcounter == prevflushcounter + 1) {
        didaflushtrigger = 1;
      } else {
        didaflushtrigger = 0;
      }
    prevflushcounter = flushcounter;
    
    updateSerial(totalTimeinSeconds, occlusionSensorValue, drainageVolume, salineVolume, didaflushtrigger);

    // LED logic
    if (redLEDActive) {
      digitalWrite(greenPin, HIGH);
      digitalWrite(orangePin, HIGH);
    } else {
      if (!orangeLEDActive && totalTimeinSeconds >= 42 && occlusionSensorValue <= threshold) {
        orangeLEDStartTime = millis();
        orangeLEDActive = true;
        digitalWrite(greenPin, HIGH);
      }
    }

    // Orange LED operation
    if (orangeLEDActive) {
      unsigned long elapsedTime = millis() - orangeLEDStartTime;

      if (elapsedTime < orangeLEDTotalDuration) {
        digitalWrite(orangePin, LOW);
        clog_yn = true; 
      } 

      if (elapsedTime >= orangeLEDTotalDuration) {
        digitalWrite(orangePin, HIGH);
        clog_yn = false;
        orangeLEDActive = false;
        orangeLEDOffCount++;
        digitalWrite(greenPin, LOW);
      }
    }

    // Orange LED tracking logic
    if (orangeLEDOffCount == 1 && !OneCountDone) {
      twiceLEDStartTime = millis();
      OneCount = true;
      OneCountDone = true;
    }

    if (orangeLEDOffCount == 2 && OneCount && !TwoCountDone) {
      twiceLEDEndTime = millis();
      TwoCount = true;
      TwoCountDone = true;
    }

    if (TwoCount) {
      timeBetween = (twiceLEDEndTime - twiceLEDStartTime) - 10000;
    }

    // After the orange LED has been turned off twice, activate the red LED
      if (timeBetween <= TimeBetweenDuration && TwoCount && !redLEDActive) {
        digitalWrite(redPin, LOW);  // Turn on red LED
        redLEDActive = true;  // Set flag to track red LED status
      } else {
        orangeLEDOffCount = 0;
        OneCount = false;
        OneCountDone = false;
        TwoCount = false;
        TwoCountDone = false;
      }
    }

    // Green LED default (on when idle)
    if (occlusionSensorValue > threshold && !redLEDActive && !orangeLEDActive) {
      digitalWrite(greenPin, LOW);
    } else {
      digitalWrite(greenPin, HIGH);
    }

    newDataReady = false;
    t = millis();
  }

  ////////////////////////////////////////////////////////////
  ///////////// NOVIRAD MAIN LOOP FUNCTION ///////////////////
  ////////////////////////////////////////////////////////////

  // CHECK FOR CLOG FUNCTION, then turn clog_yn flag to true  //
  // Handle pump logic based on sensor readings and states
  handle_pump_logic();

}

void updateSerial(float totalTimeinSeconds, float occlusionSensorValue, float drainageVolume, float salineVolume, int flushTimes) { // Display elapsed time

  Serial.print(totalTimeinSeconds, 3);
  Serial.print(", ");
  Serial.print(occlusionSensorValue);
  Serial.print(", ");
  Serial.print(salineVolume);
  Serial.print(", ");
  Serial.print(drainageVolume);
  Serial.print(", ");
  Serial.println(flushTimes);

}

////////////////////////////////////////////////////////////
///////////////// PUMP LOGIC FUNCTION //////////////////////
////////////////////////////////////////////////////////////

void handle_pump_logic() {

  ////////////////////////////////////////
  // Normal operation (no clog, no flush)
  if (!flush_now && !clog_yn && !red_led_stop) {
    //Serial.println("normal drain");
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
    abscess_flush_timer_start = millis();
    abscess_flush_timer_limit = flush_duration * 1000; // Wait for stabilization

    //Serial.print("CLOG FLUSHING FOR: ");
    //Serial.print(flush_duration);
    //Serial.println(" seconds");
    while (millis() - abscess_flush_timer_start < abscess_flush_timer_limit) {
      //Serial.println("Reversing pump and flush to clear clog");
    } 
      // Flush completed
      flush_now = false;
      clog_yn = false;
      totalflushtime += flush_duration; //Log flush duration
      //Serial.println("CLOG FLUSHING COMPLETED!");
      flushcounter++;
      //Serial.println(flushcounter);


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

int handle_flush_cycle() { 
  unsigned long current_time = millis();
  //Serial.print("Time elapsed since last flush: ");
  //Serial.println(current_time - flush_timer_start);
  
  if (current_time - flush_timer_start >= flush_frequency * 60.0 * 1000.0) {   
    //ln("Starting periodic flush cycle...");
    // Start the flush cycle (stop abscess, start saline)
    stopAbscess();
    startSalinePropulsion();
    
    // Wait for the flush duration to complete
    if (current_time - flush_timer_start >= flush_frequency * 60.0 * 1000.0 + flush_duration * 1000.0) {
      // Reset timer after flush cycle
      setup_timer();
      totalflushtime += flush_duration; // Log flush duration
      //Serial.println("Flush cycle completed.");
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
    //Serial.println("Prompted immediate flush of the system");
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
  //Serial.println("stop saline happening");
}

void startAbscessSuction() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  //Serial.println("start abscess happening");
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