#include <Wire.h>
#include <LiquidCrystal.h>    // For the parallel connection LCD
#include "HX711.h"            // For the load cell
#include <Adafruit_GFX.h>     // For the OLED display
#include <Adafruit_SSD1306.h> // For the OLED display

// Settings for HX711 load cell
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
#define CALIBRATION_FACTOR -408.7

// OLED Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// HX711 object
HX711 scale;

// LCD setup
// LiquidCrystal lcd(12, 11, 5, 6, 7, 8);

// Timer for weight updates
unsigned long lastWeightUpdate = 0; // Tracks the last weight update time
#define WEIGHT_UPDATE_INTERVAL 1000 // 1 second interval for weight updates

// Pins for LEDs and button
const byte redPin = 12, bluePin = 11, greenPin = 10, whitePin = 7;  // Pins for LEDs
const byte buttonPin = 6;  // Pin for button
int sensor = 0;
int baseline = 400;  // Default value before baseline is calculated
unsigned long blueLEDStartTime = 0;  // To track when blue LED is turned on
const long blueLEDTotalDuration = 10000;  // Total blue LED duration (10 seconds)
const long blueLEDOnDuration = 5000;  // Blue LED stays on for the first 5 seconds
const long blueLEDFlashDuration = 500;  // Flash interval (500ms)
bool blueLEDActive = false;  // Flag to check if blue LED is active
bool redLEDActive = false;  // Flag to check if red LED is active
bool whiteLEDActive = false;  // Flag to check if white LED is active
bool greenLEDActive = false;  // Flag to check if green LED is active

unsigned long previousMillis = 0;  // Store the last time the sensor was read
const long interval = 1000;  // 1 second interval (in milliseconds)

int blueLEDCount = 0;  // Counter to track blue LED activations
int blueLEDOffCount = 0;  // Counter for how many times the blue LED has been turned off

// Variables for baseline calculation
unsigned long baselineStartTime = 0;  // Start time for baseline calculation
const long baselineDuration = 60000;  // 1 minute to calculate baseline (60000 ms)
float sensorReadings[60];  // Array to store the first 60 sensor readings
int readingCount = 0;  // Counter for the number of readings taken
bool baselineCalculated = false;  // Flag to indicate that baseline has been calculated

// Function to update OLED with weight and time
void displayWeightAndTime(float weight, unsigned long currentTime) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Display weight
  display.setCursor(0, 10);
  display.println("Weight:");
  display.setCursor(0, 30);
  display.setTextSize(2);
  display.print(weight, 4); // Show 4 decimal places
  display.print(" g");

  // Display time in millis
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print("Time: ");
  display.print(currentTime);
  display.print(" ms");

  display.display();
}

void setup() {
  // Serial monitor
  Serial.begin(115200);
  Serial.println("Setup started"); // Debugging statement

  // OLED initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    Serial.println("OLED initialization failed");
    for (;;); // Halt if OLED initialization fails
  }
  display.clearDisplay();
  Serial.println("OLED initialized"); // Debugging statement

  // HX711 initialization
  Serial.println("Initializing the scale");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare();
  Serial.println("Scale initialized"); // Debugging statement

  // Initialize LED pins
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(whitePin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);  // Configure button pin as input with internal pull-up resistor

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
      float weight = scale.get_units(10);  // Read weight from HX711 (average of 10 readings)
      sensorReadings[readingCount] = weight;  // Store the weight reading
      readingCount++;

      // Print the current sensor value (weight) for debugging
      Serial.print("Baseline sensor reading: ");
      Serial.println(weight, 4);  // Print sensor value with 4 decimal places
    }
  } else if (!baselineCalculated) {
    // Calculate the average of the first 60 readings to set as the baseline
    if (readingCount == 60) {
      float sum = 0;
      for (int i = 0; i < 60; i++) {
        sum += sensorReadings[i];
      }
      baseline = sum / 60;  // Calculate the average as the baseline
      Serial.print("Baseline calculated: ");
      Serial.println(baseline, 4);  // Print baseline with 4 decimal places
      baselineCalculated = true;  // Set flag to true after baseline is calculated

      // Turn off white LED after baseline is calculated
      digitalWrite(whitePin, LOW);  // Turn off white LED
    }
  }

  // ** Green LED will only be controlled after baseline is calculated **
  if (baselineCalculated) {
    float threshold = baseline * 0.80;  // Set the threshold to 80% of baseline
    float weight = scale.get_units(10); // Get weight reading from HX711

    if (weight >= threshold) {
      digitalWrite(greenPin, HIGH);  // Activate green LED if weight is above threshold
    } else {
      digitalWrite(greenPin, LOW);   // Deactivate green LED if weight is below threshold
    }
  }

  // Handle weight scale data
  if (currentMillis - lastWeightUpdate >= WEIGHT_UPDATE_INTERVAL) {
    lastWeightUpdate = currentMillis;

    if (scale.wait_ready_timeout(200)) {
      float weight = scale.get_units(10); // Average of 10 readings
      if (weight != 0) {
        displayWeightAndTime(weight, currentMillis); // Update OLED display
        Serial.print(weight, 4);  // Print weight with 4 decimal places
        Serial.print(", ");
      } else {
        displayWeightAndTime(0, currentMillis); // Display zero weight
        Serial.print("Weight: 0");
      }
    } else {
      displayWeightAndTime(0, currentMillis); // Display scale not ready
      Serial.print("Weight: 0 (Scale not ready)");
    }

    // Convert current time to seconds and print it
    float currentTimeinSeconds = currentMillis / 1000.0; // Corrected line
    Serial.println(currentTimeinSeconds);
  }
}
