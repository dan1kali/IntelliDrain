/*
NoviDrain (Portable and Programmable Percutaneous Drainage Catheter System)
Purpose: Provide the logic for the bluetooth, pumps, and pressure sensors 
to drain and flush abnormal fluid collections in the body

Developed by Linh Vu 1/21/19

*/
/**********************************************************
  Pump Control System with Bluetooth Communication
***********************************************************/

#include "sec_func.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <SoftwareSerial.h>


////////////////////////////////////////////////////////////

//Novirad Device parameters

int saline_rpm = 220; //Saline pump rpm: scaled 0 -255
int abscess_rpm = 170;   //Saline pump rpm: scaled 0 -255
int flush_duration = 10; //Saline flush duration: seconds
float flush_frequency = 10000; //Saline flush frequency: minutes
bool bleyn = false; //true: use bluetooth app connection, false ignore bluetooth app
bool verbosex = true;
////////////////////////////////////////////////////////////

// Pin assignments
int in1 = 10, in2 = 11, in3 = 4, in4 = 5, pinSaline = 3,pinAbscess = 9;
int bt_RX = 2, bt_TX = 6;
int pinPwrA = A2, pinPwrB = A3;
int voltage_A_pin = A0;
int command_in_pin = 13;

//Tracking metrics
float totalflushvol = 0.0, totalflushtime = 0.0;

// Timers
unsigned long absess_flush_timer_start = 0; //Flush performed when clog is detected
unsigned long absess_flush_timer_limit = 0;
unsigned long flush_timer_start = 0; //Flush performed when periodic flush is performed
unsigned long flush_timer_limit = 0;
bool timer_enabled = true;

// Flags and states
bool clog_yn = false, flush_now = false;
int count = 0;
bool RPwrstate = 0; //Assign variables
bool LPwrstate = 0;
int lastButtonState = -99;
int spdtstate = 0; // 1 = Normal, 0 = Off, -1 = prime
int spdt_proceedyn = 0;
int flush_cycle_proceedyn = 0;


//Hardware and BLE initialization
SoftwareSerial bluefruitSerial(bt_TX, bt_RX);
Adafruit_BluefruitLE_UART ble(bluefruitSerial, BLUEFRUIT_UART_MODE_PIN, BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
extern uint8_t packetbuffer[]; //Bluetooth packet buffer
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
void printHex(const uint8_t * data, const uint32_t numBytes);
String getValue(String data, char separator, int index);


////////////////////////////////////////////////////////////
// SETUP FUNCTION
////////////////////////////////////////////////////////////

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize pump control pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(pinSaline, OUTPUT);
  pinMode(pinAbscess, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinPwrA, INPUT_PULLUP); //SPDT
  pinMode(pinPwrB, INPUT_PULLUP); //SPDT
  pinMode(command_in_pin, INPUT_PULLDOWN); //SPDT
  

  //Update pumps rpm
  update_pump_rpm();
  
  // Initialize timer and BLE setup
  setup_timer();
  
  if (bleyn){
    setup_ble(&ble);
  }
  Serial.println("Pump Control System Ready");
}

////////////////////////////////////////////////////////////
// MAIN LOOP FUNCTION
////////////////////////////////////////////////////////////

void loop() {
  
  // Check for new Bluetooth packets
  if (ble.available() && bleyn) {
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
    if (len > 0) {
      printHex(packetbuffer, len);
      process_ble_packet();
    }
  }

  //Check toggle switch status (e.g., on, prime, off) 
  spdt_proceedyn = handle_toggle_switch();
  if (spdt_proceedyn == 0) {return;}

  
  // Timer-based pump control for periodic flushing
  //flush_cycle_proceedyn = handle_flush_cycle();
  //if (flush_cycle_proceedyn == 0) {return;}

  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  // CHECK FOR CLOG FUNCTION, then turn clog_yn flag to true  //
  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////


  int signal = digitalRead(command_in_pin);  // Read the signal from pin 13
    Serial.print("command_in_pin is: ");
    Serial.println(signal);

  if (signal == HIGH) {
    clog_yn = true; 
    Serial.print("clog_yn is: ");
    Serial.println(clog_yn);

  } else {
    clog_yn = false; 
    Serial.print("clog_yn is: ");
    Serial.println(clog_yn);
  }

  // Handle pump logic based on sensor readings and states
  handle_pump_logic();
}


////////////////////////////////////////////////////////////
// HANDLE TOGGLE SWITCH
////////////////////////////////////////////////////////////
int handle_toggle_switch(){

    RPwrstate = digitalRead(pinPwrA);  //read the input pins
    LPwrstate = digitalRead(pinPwrB);
  
    if ( (RPwrstate == LOW) && (LPwrstate == HIGH)) //test for right position
    {
      spdtstate = 1; // 1 = Normal, 0 = Off, -1 = prime
      //Serial.println("Switch in Right position");
    }
    if ( (RPwrstate == HIGH) && (LPwrstate == HIGH)) //test for center position
    {
      spdtstate = 0; // 1 = Normal, 0 = Off, -1 = prime    
      //Serial.println("Switch in Center position");
    }
    if ( (RPwrstate == HIGH) && (LPwrstate == LOW))  //test for left position
    {
      spdtstate = -1; // 1 = Normal, 0 = Off, -1 = prime    
      //Serial.println("Switch in Left position");
    } 
  
    ///////////////////////////////////////////////////////////////////////////////////////////////
    
    //Once device is powered on, get current position of the switch and create last button state (runs once)
    if (lastButtonState == -99){  
      lastButtonState = spdtstate;
    }

    //Check for a change in toggle switch state
    if (spdtstate != lastButtonState && spdtstate == 1) { //Switched from off or prime to on  
        //Serial.println("Switched from prime/off position to on position");
        setup_timer();  
      }
    else if (spdtstate != lastButtonState && spdtstate == 0) { //Switched from on or prime to off
        //Serial.println("Switched from on/prime position to off position");
        spdtstate = 0;
    }

    //Update spdtstate
    lastButtonState = spdtstate;

    //Change pump settings depending on toggle switch state
    if (spdtstate == 0){ //Off position
      stopAbscess();
      stopSaline();
      return 0; //Flag to repeat main loop
    }
   else if (spdtstate == -1){ //Primes position
      stopAbscess();
      startSalinePropulsion();
      return 0; //Flag to repeat main loop
    }
   else if (spdtstate == 1){ //On position
      return 1; //Flag to proceed to remaining code
   }
  
}



////////////////////////////////////////////////////////////
// PUMP LOGIC FUNCTION
////////////////////////////////////////////////////////////

void handle_pump_logic() {

  ////////////////////////////////////////
  // Normal operation (no clog, no flush)
  if (!flush_now && !clog_yn) {
    startAbscessSuction();
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
    absess_flush_timer_start = millis();
    absess_flush_timer_limit = flush_duration * 1000; // Wait for stabilization

    // Serial.print("CLOG FLUSHING FOR: ");
    // Serial.print(flush_duration);
    // Serial.println(" seconds");
    while (millis() - absess_flush_timer_start < absess_flush_timer_limit) {
      //Serial.println("Reversing pump and flush to clear clog");
    } 
      // Flush completed
      flush_now = false;
      clog_yn = false;
      totalflushtime += flush_duration; //Log flush duration
      // Serial.println("CLOG FLUSHING COMPLETED!");

      //RESET PERIODIC TIMER
      setup_timer();

      //Stop pumps for a 1/2 second
      stopAbscess();
      stopSaline();
      //delay(500);     
  }
}

////////////////////////////////////////////////////////////
// FLUSH CYCLE START FUNCTION
////////////////////////////////////////////////////////////


/*

// changed millis() to current time, added first Serial.printIn
// --> only brief hiccup after 60s, no flush. Still errors.

int handle_flush_cycle() {
  unsigned long current_time = millis();
 
  Serial.println(current_time - flush_timer_start);
  if (current_time - flush_timer_start >= flush_frequency*60.0*1000.0) {   
      // Serial.println("Starting flush cycle"); 
      stopAbscess();
      startSalinePropulsion();
      if (current_time - flush_timer_start > flush_frequency*60.0*1000.0 + flush_duration*1000.0){ //After flush duration is completed, restore normal function
        //Flush cycle completed
        setup_timer(); //Reset time
        totalflushtime += flush_duration; //Log flush duration
      return 1;
      }
      else { //Flush is not completed. Repeat loop
      return 0;
      }
  }
  return 1;
}

*/


// working version, no one knows why
int handle_flush_cycle() { 
  unsigned long current_time = millis();
  // Serial.print("Time elapsed since last flush: ");
  // Serial.println(current_time - flush_timer_start);
  
  if (current_time - flush_timer_start >= flush_frequency * 60.0 * 1000.0) {   
    // Serial.println("Starting periodic flush cycle...");
    // Start the flush cycle (stop abscess, start saline)
    stopAbscess();
    startSalinePropulsion();
    
    // Wait for the flush duration to complete
    if (current_time - flush_timer_start >= flush_frequency * 60.0 * 1000.0 + flush_duration * 1000.0) {
      // Reset timer after flush cycle
      setup_timer();
      totalflushtime += flush_duration; // Log flush duration
      // Serial.println("Flush cycle completed.");
      return 1;
    } else {
      // Flush not yet completed
      return 0;
    }
  }
  return 1; // Default return value if time condition is not met
}

////////////////////////////////////////////////////////////
// BLUETOOTH PACKET HANDLING
////////////////////////////////////////////////////////////

void process_ble_packet() {
  // Extract Bluetooth commands and update pump parameters
  if (packetbuffer[1] == 'B') {
    int button_num = packetbuffer[2] - '0';
    if (button_num == 9) {
      // Parse and update power and flush settings from BLE data
      String abscess_power = getValue(packetbuffer, 'x', 1);
      String flush_power = getValue(packetbuffer, 'x', 2);
      String flush_volume = getValue(packetbuffer, 'x', 3);
      String flush_freq = getValue(packetbuffer, 'x', 4);
      String auto_flush = getValue(packetbuffer, 'x', 5);

      update_settings(abscess_power, flush_power, flush_volume, flush_freq, auto_flush);
    }
  }
}




////////////////////////////////////////////////////////////
// TIMER FUNCTION
////////////////////////////////////////////////////////////

void setup_timer() {
  flush_timer_start = millis();

}

////////////////////////////////////////////////////////////
// UPDATE SETTINGS FUNCTION
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
    // Serial.println("Prompted immediate flush of the system");
    flush_now = true;    
  }

  update_pump_rpm();
  setup_timer();
}

////////////////////////////////////////////////////////////
// PUMP CONTROL FUNCTION
////////////////////////////////////////////////////////////

void update_pump_rpm() {
  analogWrite(pinSaline, saline_rpm);
  analogWrite(pinAbscess, abscess_rpm);
}

////////////////////////////////////////////////////////////
// PUMP CONTROL HELPERS
////////////////////////////////////////////////////////////

void stopAbscess() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void stopSaline() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void startAbscessSuction() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
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
        // Serial.println(message);
    }
}
