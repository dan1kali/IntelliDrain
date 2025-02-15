const byte redPin = 8, bluePin = 12, greenPin = 13, whitePin = 4;  // Pins for LEDs
const byte buttonPin = 3;  // Pin for button
int sensor = 0;
int baseline = 400;  // Default value before baseline is calculated
unsigned long blueLEDStartTime = 0;  // To track when blue LED is turned on
const long blueLEDTotalDuration = 10000;  // Total blue LED duration (10 seconds)
const long blueLEDOnDuration = 5000;  // Blue LED stays on for the first 5 seconds
const long blueLEDFlashDuration = 500;  // Flash interval (500ms)
bool blueLEDActive = false;  // Flag to check if blue LED is active
bool blueLEDFlashing = false;  // Flag to track if blue LED should be flashing

unsigned long previousMillis = 0;  // Store the last time the sensor was read
const long interval = 1000;  // 1 second interval (in milliseconds)

int blueLEDCount = 0;  // Counter to track blue LED activations
bool redLEDActive = false;  // Flag to check if red LED is active

// Counter for how many times the blue LED has been turned off
int blueLEDOffCount = 0;

// Variables for baseline calculation
unsigned long baselineStartTime = 0;  // Start time for baseline calculation
const long baselineDuration = 60000;  // 1 minute to calculate baseline (60000 ms)
int sensorReadings[60];  // Array to store the first 60 sensor readings
int readingCount = 0;  // Counter for the number of readings taken
bool baselineCalculated = false;  // Flag to indicate that baseline has been calculated

void setup() {
  pinMode(A0, INPUT);  // Sensor input pin
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(whitePin, OUTPUT);  // Set the white LED pin as output
  pinMode(buttonPin, INPUT_PULLUP);  // Configure button pin as input with internal pull-up resistor
  Serial.begin(9600);
  
  // Start with white LED on during baseline calculation
  digitalWrite(whitePin, HIGH);  // Turn on white LED
  digitalWrite(redPin, LOW);  // Ensure other LEDs are off initially
  digitalWrite(bluePin, LOW);  
  digitalWrite(greenPin, LOW);  
  
  Serial.println("System on!");
  
  baselineStartTime = millis();  // Record the start time for baseline calculation
}

void loop() {
  unsigned long currentMillis = millis();  // Get the current time
  
  // If the baseline has not been calculated yet (first 60 seconds)
  if (currentMillis - baselineStartTime < baselineDuration && !baselineCalculated) {
    // Collect sensor readings for the first minute (1 reading per second)
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      sensor = analogRead(A0);  // Read the sensor value
      sensorReadings[readingCount] = sensor;  // Store the sensor reading
      readingCount++;

      // Print the current sensor value for debugging
      Serial.print("Baseline sensor reading: ");
      Serial.println(sensor);
    }
  } else if (!baselineCalculated) {
    // Calculate the average of the first 60 readings to set as the baseline
    if (readingCount == 60) {
      int sum = 0;
      for (int i = 0; i < 60; i++) {
        sum += sensorReadings[i];
      }
      baseline = sum / 60;  // Calculate the average as the baseline
      Serial.print("Baseline calculated: ");
      Serial.println(baseline);
      baselineCalculated = true;  // Set flag to true after baseline is calculated
      
      // Turn off white LED after baseline is calculated
      digitalWrite(whitePin, LOW);  // Turn off white LED
      digitalWrite(greenPin, HIGH); // Keep green LED on after baseline is calculated
    }
  }

  // Proceed with the regular behavior after the baseline has been calculated
  // If the red LED is active, skip reading the sensor value and turn off all LEDs
  if (redLEDActive) {
    digitalWrite(greenPin, LOW);  // Turn off green LED
    digitalWrite(bluePin, LOW);   // Turn off blue LED

    // Check for button press to turn off red LED
    if (digitalRead(buttonPin) == HIGH) {  // Button is pressed (assuming button is connected to ground)
      digitalWrite(redPin, LOW);  // Turn off red LED
      redLEDActive = false;  // Reset the red LED active flag
      digitalWrite(greenPin, HIGH);  // Reactivate green LED
      blueLEDOffCount = 0;  // Reset the blue LED off counter
      Serial.println("Red LED turned off. Resume.");
    }
  } else {
    // Check if the blue LED is not active (i.e., green LED should remain on)
    if (!blueLEDActive) {
      // Check if 1 second has passed since the last sensor reading
      if (currentMillis - previousMillis >= interval) {
        // Save the last time we evaluated the sensor
        previousMillis = currentMillis;
        
        // Read the sensor value
        sensor = analogRead(A0);

        // Calculate 80% of baseline (20% less than baseline)
        float threshold = baseline * 0.80;  // 20% less than baseline

        // If the sensor value is less than 80% of the baseline, activate the blue LED for 10 seconds
        if (sensor < threshold) {
          if (!blueLEDActive) {  // Only activate if the blue LED is not already on
            digitalWrite(bluePin, HIGH);  // Turn on blue LED
            blueLEDStartTime = millis();  // Record the start time
            blueLEDActive = true;  // Set the flag to true to track the blue LED timing
            digitalWrite(greenPin, LOW);  // Turn off green LED when blue is activated

            // Increment the blue LED activation counter
            blueLEDCount++;
          }
        }
        
        // Print sensor value for debugging
        Serial.print("Sensor = ");
        Serial.println(sensor);
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
    }
    // Last 5 seconds: Blue LED flashes
    else if (elapsedTime >= blueLEDOnDuration && elapsedTime < blueLEDTotalDuration) {
      // Flash blue LED every 500 milliseconds
      if (elapsedTime % blueLEDFlashDuration < blueLEDFlashDuration / 2) {
        digitalWrite(bluePin, HIGH);  // Turn on blue LED
      } else {
        digitalWrite(bluePin, LOW);  // Turn off blue LED
      }
    }

    // After 10 seconds, turn off the blue LED and restore green LED
    if (elapsedTime >= blueLEDTotalDuration) {
      digitalWrite(bluePin, LOW);  // Turn off blue LED
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

  // ** New logic for green LED to stay on when sensor value is above threshold **
  if (baselineCalculated) {
    // If the sensor value is above 80% of baseline, keep the green LED on
    float threshold = baseline * 0.80;  // 80% of the baseline
    if (sensor >= threshold) {
      digitalWrite(greenPin, HIGH);  // Keep green LED on
    } else {
      digitalWrite(greenPin, LOW);   // Turn off green LED if sensor value falls below threshold
    }
  }
}
