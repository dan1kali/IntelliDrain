#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED width, in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels
#define OLED_RESET     4 // Reset pin (optional, can be omitted for I2C)
#define SCREEN_ADDRESS 0x3D // I2C address for SSD1306 OLED (0x3C or 0x3D)

const int sensorPin = A0; // Pressure sensor connected to A0
const float alpha = 0.95; // Low-pass filter alpha (0 - 1 )
const float aRef = 4.6; // analog reference
float filteredVal = 512.0; // midway starting point for filter
float sensorVal;
float voltage;
float psiVal;

// Create an instance of the Adafruit_SSD1306 display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600); // Begin serial communication
  while (!Serial); // Wait for serial to initialize

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1); // Halt if display initialization fails
  }

  // Flashing heart at the beginning
  flashStar();

  // Clear the display
  display.clearDisplay();
  display.display();
}

void loop() {
  // Read the sensor value from A0 and apply low-pass filter
  sensorVal = (float)analogRead(sensorPin); 
  filteredVal = (alpha * filteredVal) + ((1.0 - alpha) * sensorVal); // Low-pass filter
  voltage = (filteredVal / 1024.0) * aRef; // Convert to voltage
  psiVal = (voltage - 0.4784) / 0.0401; // Convert voltage to psi using a formula

  // Print raw, filtered ADC values to the serial monitor
  Serial.print("raw_adc: ");
  Serial.print(sensorVal, 0);
  Serial.print(", filtered_adc: ");
  Serial.println(filteredVal, 0);

  // Update the OLED with the latest sensor data
  updateOLED();

  delay(200); // Delay for 200ms before the next reading
}

void flashStar() {
  for (int i = 0; i < 5; i++) { // Flash star 5 times
    drawStar(); // Draw the star
    delay(500); // Wait for 500ms
    display.clearDisplay(); // Clear the display
    display.display();
    delay(500); // Wait for 500ms
  }
}

void drawStar() {
  display.setTextSize(1);  // Set text size to 1 (small text)
  display.setTextColor(WHITE); // Set text color to white
  
  // Clear the display before drawing the heart
  display.clearDisplay();

  // Define center of heart
  int centerX = 64;  // Center X coordinate (half of 128)
  int centerY = 32;  // Center Y coordinate (half of 64)

  // Draw the heart using triangles
  display.fillCircle(40, 32, 15, SSD1306_WHITE);  // Left top part
  display.fillCircle(80, 32, 15, SSD1306_WHITE);  // Right top part
  display.fillTriangle(20, 32, 60, 50, 100, 32, SSD1306_WHITE);  // Bottom triangle part

  // Render the display
  display.display();
}




void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);  // Set text size for smaller text
  display.setTextColor(SSD1306_WHITE);  // Set text color to white

  display.setCursor(0, 0);
  display.print("filtered_adc ");
  display.print(filteredVal, 0);  // Print filtered ADC value without decimal places

  display.setCursor(0, 15);
  display.print("voltage ");
  display.print(voltage, 2);  // Print voltage with 2 decimal places
  display.print(" Volts");

  display.setTextSize(3);  // Set text size for PSI value to be large
  display.setCursor(0, 30);

  // Print PSI with 1 decimal place
  display.print(psiVal, 1);  // 1 decimal place

  display.setTextSize(2);  // Set text size for PSI value to be large
  display.print(" psi");
  display.display();  // Update the OLED display with new content
}



