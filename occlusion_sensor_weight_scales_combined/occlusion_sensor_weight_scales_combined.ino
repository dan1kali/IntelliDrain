#include <Wire.h>
#include <LiquidCrystal.h>    // For the parallel connection LCD
#include <Adafruit_GFX.h>     // For the OLED display
#include <Adafruit_SSD1306.h> // For the OLED display

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif


// Occlusion sensor load cell setup
const int LOADCELL_SCK_PIN = 3;
const int LOADCELL_DOUT_PIN = 2;
#define CALIBRATION_FACTOR -408.7

// HX711 object
HX711_ADC LoadCell_0(LOADCELL_SCK_PIN, LOADCELL_DOUT_PIN); //HX711 0

//Weight scale setup pins:
const int weightscale_HX711_sck_1 = 5; //mcu > HX711 no 1 sck pin
const int weightscale_HX711_dout_1 = 4; //mcu > HX711 no 1 dout pin
const int weightscale_HX711_sck_2 = 9; //mcu > HX711 no 2 sck pin
const int weightscale_HX711_dout_2 = 8; //mcu > HX711 no 2 dout pin

//HX711 constructor for weight scales (dout pin, sck pin)
HX711_ADC LoadCell_1(weightscale_HX711_dout_1, weightscale_HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(weightscale_HX711_dout_2, weightscale_HX711_sck_2); //HX711 2

const int calVal_eepromAdress_1 = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom adress for calibration value load cell 2 (4 bytes)
unsigned long t = 0;
unsigned long startMillis; // To store the start time


// Timer for weight updates
const int updateInterval = 1000; //How often data is collected in milliseconds

// OLED Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Function to update OLED with weight
void displayWeight(float weight) {
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

  display.display();
}

void setup() {
  // Serial monitor
  Serial.begin(115200);
  Serial.println("\nStarting...");


  // OLED initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    Serial.println("OLED initialization failed");
    for (;;); // Halt if OLED initialization fails
  }
  display.clearDisplay();
  Serial.println("OLED initialized"); // Debugging statement

  //Serial.begin(57600);

  float calibrationValue_0; // calibration value load cell 0
  float calibrationValue_1; // calibration value load cell 1
  float calibrationValue_2; // calibration value load cell 2

  calibrationValue_0 = -408.7; // uncomment this if you want to set this value in the sketch
  calibrationValue_1 = -200.43; // uncomment this if you want to set this value in the sketch
  calibrationValue_2 = -216.33; // uncomment this if you want to set this value in the sketch
  #if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
  #endif
  //EEPROM.get(calVal_eepromAdress_1, calibrationValue_1); // uncomment this if you want to fetch the value from eeprom
  //EEPROM.get(calVal_eepromAdress_2, calibrationValue_2); // uncomment this if you want to fetch the value from eeprom

  LoadCell_0.begin();
  LoadCell_1.begin();
  LoadCell_2.begin();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte loadcell_0_rdy = 0;   byte loadcell_1_rdy = 0;  byte loadcell_2_rdy = 0;
  while ((loadcell_0_rdy + loadcell_1_rdy + loadcell_2_rdy) < 3) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_0_rdy) loadcell_0_rdy = LoadCell_0.startMultiple(stabilizingtime, _tare);
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_0.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.0 wiring and pin designations (occlusion sensor)");
  }
   if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations (weight scale 1)");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations (weight scale 2)");
  }

  startMillis = millis();


  LoadCell_0.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  Serial.println("Weight scales startup is complete");
}


///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LOOP ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////



void loop() {
  unsigned long currentTime = millis();
  static boolean newDataReady = 0;

  // check for new data/start next conversion:
  if (LoadCell_0.update()) newDataReady = true;
  LoadCell_1.update();
  LoadCell_2.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t + updateInterval) {

      unsigned long currentMillis = millis();
      float totalTimeinSeconds = (currentMillis - startMillis) / 1000.0; // Corrected line
      Serial.print(totalTimeinSeconds, 3);
      Serial.print(", ");


      float weight0 = LoadCell_0.get_units(10); // Average of 10 readings
      displayWeight(weight0); // Update OLED display
      Serial.print(weight0);
      Serial.print(", ");
      float weight1 = LoadCell_1.get_units(10);
      float weight2 = LoadCell_2.get_units(10);
      Serial.print(weight1);
      Serial.print(", ");
      Serial.println(weight2);

      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
    }
  }

  //check if last tare operation is complete
  if (LoadCell_1.getTareStatus() == true) {
    //Serial.println("Tare load cell 1 complete");
  }
  if (LoadCell_2.getTareStatus() == true) {
    //Serial.println("Tare load cell 2 complete");
  }


}
