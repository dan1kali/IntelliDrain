
///// LIBRARY INCLUSIONS /////
#include <HX711_ADC.h> // Communicates with HX711 load cell amplifier module

// Your WiFi credentials. Set password to "" for open networks. ***NEEDED?***
char ssid[] = "Rice Visitor";
char pass[] = "";

///// PIN DEFINITIONS /////
const int LOADCELL_SCK_PIN = 7; // Sensor serial clock
const int LOADCELL_DOUT_PIN = 6; // Sensor load cell data
const byte redPin = 5, orangePin = 4, greenPin = 3;  // Pins for LEDs
const byte buttonResetPin = 1;  // Pin for reset button
const byte buttonFlushPin = 0; // Pin for flush button
const int command_out_pin = 2; // Output to send flushing command

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
  ///// SERIAL COMMUNICATION INITIALIZATION /////
  delay(3000);
  Serial.begin(9600);
  Serial.println("Starting...");

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
  digitalWrite(greenPin, LOW);  /// Start with green LED
  digitalWrite(command_out_pin, LOW);  
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
    digitalWrite(greenPin, LOW);
    orangeLEDOffCount = 0;
    OneCount = false;
    OneCountDone = false;
    TwoCount = false;
    TwoCountDone = false;
    Serial.println("Red LED turned off. Resume.");
    delay(200);
  }

  if (newDataReady && currentMillis > t + updateInterval) {
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
      delay(200); // debounce
    }

    // Orange LED operation
    if (orangeLEDActive) {
      unsigned long elapsedTime = millis() - orangeLEDStartTime;

      if (elapsedTime < orangeLEDTotalDuration) {
        digitalWrite(orangePin, LOW);
        digitalWrite(command_out_pin, HIGH);
        Serial.println("command out pin high");
      } 

      if (elapsedTime >= orangeLEDTotalDuration) {
        digitalWrite(orangePin, HIGH);
        digitalWrite(command_out_pin, LOW);
        Serial.println("command out pin low");
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
}



void updateSerial(float totalTimeinSeconds, float weight0) { // Display elapsed time

  Serial.print(totalTimeinSeconds, 3);
  Serial.print(", ");
  Serial.println(weight0);


}


