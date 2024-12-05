#include <Wire.h>
#include <LiquidCrystal.h>    // For the parallel connection LCD
#include "HX711.h"            // For the load cell
#include <Adafruit_GFX.h>     // For the OLED display
#include <Adafruit_SSD1306.h> // For the OLED display
#include <Pushbutton.h>       // For the button debounce

// Settings for water flow sensor
#define BTN_PIN             9    // Button pin
#define SAMPLING_DELAY      500  // Delay between samples (ms)
#define DEBOUNCE_DELAY      30   // Button debounce delay (ms)
#define MODE_IDLE           0    // Idle mode
#define MODE_RECORDING      1    // Recording mode
#define NUM_MODES           2    // Total modes

// Settings for HX711 load cell
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
#define CALIBRATION_FACTOR -408.7

// OLED Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// HX711 object
HX711 scale;

// LCD setup
LiquidCrystal lcd(12, 11, 5, 6, 7, 8);

// Button for HX711
Pushbutton button(4);

// Water flow sensor
const int flowSensorPin = 19; // Pin 19 for water flow sensor (interrupt 4)
volatile int pulseCount = 0;
unsigned int flowRate = 0;
unsigned long lastSampleTime = 0;

// Mode tracking
uint8_t mode = MODE_IDLE;
uint8_t btn_state = HIGH;
uint8_t last_btn_state = HIGH;
unsigned long last_dbnc_time = 0;

// Flag to stop serial communication
bool serialActive = true;  // Initially serial is active

// Function to update OLED with weight
void displayWeight(float weight) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Weight:");
  display.setCursor(0, 30);
  display.setTextSize(2);
  display.print(weight, 4); // Show 4 decimal places
  display.print(" g");
  display.display();
}

// Interrupt service routine for water flow sensor
void detectRisingEdge() {
  pulseCount++;
}

void setup() {
  // Serial monitor
  Serial.begin(115200);
  
  // LCD initialization
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IDLE");

  // OLED initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    for (;;); // Halt if OLED initialization fails
  }
  display.clearDisplay();

  // HX711 initialization
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);
  scale.tare();

  // Button initialization
  pinMode(BTN_PIN, INPUT_PULLUP);

  // Water flow sensor setup
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), detectRisingEdge, RISING);
}

void loop() {
  // Check for serial input to stop communication
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 's') {
      serialActive = false;  // Stop serial communication
    }
  }

  // If serial communication is active, continue sending data
  if (serialActive) {
    // Handle mode switching via button press (unchanged)
    int btn_reading = digitalRead(BTN_PIN);
    if (btn_reading != last_btn_state) {
      last_dbnc_time = millis();
    }
    if ((millis() - last_dbnc_time) > DEBOUNCE_DELAY) {
      if (btn_reading != btn_state) {
        btn_state = btn_reading;
        if (btn_state == LOW) {
          mode = (mode + 1) % NUM_MODES;
          lcd.clear();
          lcd.setCursor(0, 0);
          if (mode == MODE_IDLE) {
            lcd.print("IDLE");
          } else if (mode == MODE_RECORDING) {
            lcd.print("Recording");
          }
        }
      }
    }
    last_btn_state = btn_reading;

    // Handle scale and water flow sensor data (unchanged)
    if (scale.wait_ready_timeout(200)) {
      float weight = scale.get_units(10); // Average of 10 readings
      if (weight != 0) {
        displayWeight(weight); // Update OLED display
        Serial.print(weight);  // Send the weight value
      } else {
        Serial.print("0");    // Default to 0 if no weight detected
      }
    } else {
      Serial.print("0");        // Default to 0 if the scale isn't ready
    }

    // Add a delimiter between the variables
    Serial.print(",");             // Use a comma to separate weight and flow rate

    // Send flow rate
    Serial.println(flowRate);      // Send the flow rate value to the Serial Plotter

    // Handle water flow sensor data (unchanged)
    if (mode == MODE_RECORDING) {
      unsigned long currentTime = millis();
      if (currentTime - lastSampleTime >= SAMPLING_DELAY) {
        lastSampleTime = currentTime;
        flowRate = (pulseCount * 180000 / 5880); // Calculate flow rate in mL/min
        pulseCount = 0;

        // Update LCD with flow rate
        lcd.setCursor(0, 0);
        lcd.print("Flow Rate:");
        lcd.setCursor(0, 1);
        lcd.print(flowRate);
        lcd.print(" mL/min");
      }
    }
  }

  // Handle HX711 data (unchanged)
  if (scale.wait_ready_timeout(200)) {
    float reading = scale.get_units(10); // Average of 10 readings
    if (reading != 0) {
      displayWeight(reading);
    }
  } else {
    Serial.println("HX711 not found.");
  }

  // Handle button press for HX711 tare (unchanged)
  if (button.getSingleDebouncedPress()) {
    scale.tare();
  }
}
