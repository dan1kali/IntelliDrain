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
LiquidCrystal lcd(12, 11, 5, 6, 7, 8);

// Timer for weight updates
unsigned long lastWeightUpdate = 0; // Tracks the last weight update time
#define WEIGHT_UPDATE_INTERVAL 1000 // 1 second interval for weight updates

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

  // LCD initialization
  lcd.begin(16, 2);
  Serial.println("LCD initialized"); // Debugging statement
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Flow Rate:");
  lcd.setCursor(0, 1);
  lcd.print("0 mL/min");

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
}

void loop() {
  unsigned long currentTime = millis();

  // Handle weight scale data
  if (currentTime - lastWeightUpdate >= WEIGHT_UPDATE_INTERVAL) {
    lastWeightUpdate = currentTime;

    if (scale.wait_ready_timeout(200)) {
      float weight = scale.get_units(10); // Average of 10 readings
      if (weight != 0) {
        displayWeightAndTime(weight, currentTime); // Update OLED display
        Serial.print(weight);
        Serial.print(", ");
      } else {
        displayWeightAndTime(0, currentTime); // Display zero weight
        Serial.print("Weight: 0");
      }
    } else {
      displayWeightAndTime(0, currentTime); // Display scale not ready
      Serial.print("Weight: 0 (Scale not ready)");
    }

    // Convert current time to seconds and print it
    float currentTimeinSeconds = currentTime / 1000.0; // Corrected line
    Serial.println(currentTimeinSeconds);
  }
}
