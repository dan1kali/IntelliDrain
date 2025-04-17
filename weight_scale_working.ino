
#include <HX711_ADC.h> // Communicates with HX711 load cell amplifier module

const int WEIGHTSCALE1_SCK_PIN = 8; // Weight Scale 1 SCK
const int WEIGHTSCALE1_DOUT_PIN = 7; // Weight Scale 1 DOUT

///// LOAD CELL SETUP /////
HX711_ADC LoadCell_1(WEIGHTSCALE1_DOUT_PIN, WEIGHTSCALE1_SCK_PIN); // Weight Scale 1 Load Cell

///// TIME VARIABLES /////
unsigned long t = 0;
unsigned long startMillis; // To store the start time
const int updateInterval = 954; //How often data is collected in milliseconds *********************************************

void setup() {
  ///// SERIAL COMMUNICATION INITIALIZATION /////
  Serial.begin(115200);
  Serial.println("Starting...");

  ///// CALIBRATION VALUES /////
  float calibrationValue_1 = -200.43; // Calibration value for weight scale 1 load cell

  ///// LOAD CELL INITIALIZATION /////
  LoadCell_1.begin();
  ///// TARE & STABILIZATION /////
  delay(5000);
  LoadCell_1.start(2000, true);
  
  ///// CURRENT TIME /////
  startMillis = millis(); // Stores time, in milliseconds, since the Arduino was powered on or reset (current time) ****************************

  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value for weight scale 1 (float)

}


void loop() {
  LoadCell_1.update();
  float weight1 = LoadCell_1.getData(); 
  Serial.println(weight1);
  delay(1000);
  }

