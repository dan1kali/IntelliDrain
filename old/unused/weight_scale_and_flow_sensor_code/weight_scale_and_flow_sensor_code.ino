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
  Serial.println("Setup started"); // Debugging statement

  // LCD initialization
  lcd.begin(16, 2);
  Serial.println("LCD initialized"); // Debugging statement
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IDLE");

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

  // Button initialization
  pinMode(BTN_PIN, INPUT_PULLUP);
  Serial.println("Button initialized"); // Debugging statement

  // Water flow sensor setup
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), detectRisingEdge, RISING);
  Serial.println("Water flow sensor initialized"); // Debugging statement
}

void loop() {
  // Handle button state for mode switching
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
          Serial.println("Mode switched to IDLE");
        } else if (mode == MODE_RECORDING) {
          lcd.print("Recording");
          Serial.println("Mode switched to Recording");
        }
      }
    }
  }
  last_btn_state = btn_reading;

  // Handle water flow sensor data
  if (mode == MODE_RECORDING) {
    unsigned long currentTime = millis();
    if (currentTime - lastSampleTime >= SAMPLING_DELAY) {
      lastSampleTime = currentTime;
      flowRate = (pulseCount * 240000 / 5880); // Calculate flow rate in mL/min
      pulseCount = 0;

      // Update LCD with flow rate
      lcd.setCursor(0, 0);
      lcd.print("Flow Rate:");
      lcd.setCursor(0, 1);
      lcd.print(flowRate);
      lcd.print(" mL/min");

      // Debugging info
      Serial.print("Flow Rate: ");
      Serial.print(flowRate);
      Serial.println(" mL/min");
    }
  }

  // Handle HX711 data
  if (scale.wait_ready_timeout(200)) {
    float reading = scale.get_units(10); // Average of 10 readings
    if (reading != 0) {
      displayWeight(reading);
    }
  } else {
    Serial.println("HX711 not found.");
  }

  // Handle button press for HX711 tare
  if (button.getSingleDebouncedPress()) {
    scale.tare();
    Serial.println("Tare...");
  }
}
