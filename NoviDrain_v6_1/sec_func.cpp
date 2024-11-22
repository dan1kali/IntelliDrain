#include <string.h>

#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_) && not defined(__SAMD51__)
  #include <SoftwareSerial.h>
#endif

#include "sec_func.h"
#include <Arduino.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"


#define PACKET_ACC_LEN                  (15)
#define PACKET_GYRO_LEN                 (15)
#define PACKET_MAG_LEN                  (15)
#define Puint8_t packetbuffer[READ_BUFSIZE+1];ACKET_QUAT_LEN                 (19)
#define PACKET_BUTTON_LEN               (5)
#define PACKET_COLOR_LEN                (6)
#define PACKET_LOCATION_LEN             (15)

//    READ_BUFSIZE            Size of the read buffer for incoming packets
#define READ_BUFSIZE                    (150) //Default 20/30

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
                      
void setup_ble(Adafruit_BluefruitLE_UART *ble){  
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble->begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble->factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble->echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble->info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble->verbose(false);  // debug info is a little annoying after this point!
while (! ble->isConnected() ) {
    Serial.println("BLE device not connected!");
    delay(500);
  }
   // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble->setMode(BLUEFRUIT_MODE_DATA);
      
  }


Adafruit_BluefruitLE_UART connect_ble(Adafruit_BluefruitLE_UART ble){
  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  return ble;  
}


int check_ble_data(Adafruit_BluefruitLE_UART *ble){
  //if ble.available()
  uint8_t len = readPacket(ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return 0;
    /* Got a packet! */
  printHex(packetbuffer, len);

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print(F("Button ")); Serial.print(buttnum);
   
    if (pressed) {
      Serial.println(F(" pressed"));

      switch (buttnum){
        case 1: //Absess pump low
          //lcd.clear();
          //lcd.setCursor(0,0);
          //lcd.print("Absess rate-Low");
          break;
        case 2: //Absess pump high
          //lcd.clear();
          //lcd.setCursor(0,0);
          //lcd.print("Absess rate-High");
          break;
        case 3: //Saline pump low
          //lcd.clear();
          //lcd.setCursor(0,0);
          //lcd.print("Saline rate-Low");
          break;
        case 4: //Saline pump high
          //lcd.clear();
          //lcd.setCursor(0,0);          
          //lcd.print("Saline rate-High");
          break;
        case 5: //Flush duration short
          //lcd.clear();
          //lcd.setCursor(0,0);
          //lcd.print("Flush Len-Short");
          break;
        case 6: //Flush duration long
          //lcd.clear();
          //lcd.setCursor(0,0);
          //lcd.print("Flush Len-Long");
          break;
        case 7: //Flush frequency Auto
          //lcd.clear();
          //lcd.setCursor(0,0);
          //lcd.print("Flush Freq-Auto");
          break;
        case 8: //Flush frequency Now
          //lcd.clear();
          //lcd.setCursor(0,0);
          //lcd.print("Flushing now!");  
          //digitalWrite(ledPin, HIGH);  // turn the ledPin on
          break;
        }
           
      }
      
     else {
      Serial.println(F(" released"));
      //digitalWrite(ledPin, LOW);  // turn the ledPin on
      //lcd.setCursor(0,0);
      //lcd.print("NoviDrain App v1");  
      return buttnum;
    }
  }

  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer+2);
    lon = parsefloat(packetbuffer+6);
    alt = parsefloat(packetbuffer+10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Accel\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Mag\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }
}

int setvals_new[4] = {40,40,40,40};

void update_settingsx(int setvals[]){
   setvals[0] = 0;
   setvals[1] = 1;
   setvals[2] = 2;
   setvals[3] = 3;
  }

void update_settings(int b, int setvals[]){
 
    switch (b){
      case 0: 
        break;
      case 1: //Absess pump low
        setvals[0] = B1SELECT;
        break;
      case 2: //Absess pump high
        setvals[0] = B2SELECT;
        break;
      case 3: //Saline pump low
        setvals[1] = B3SELECT;
        break;
      case 4: //Saline pump high
        setvals[1] = B4SELECT;
        break;
      case 5: //Flush duration short
        setvals[2] = B5SELECT;
        break;
      case 6: //Flush duration long
        setvals[2] = B6SELECT;
        break;
      case 7: //Flush frequency Auto
        setvals[3] = B7SELECT;
        break;
      case 8: //Flush frequency Now
        setvals[3] = B8SELECT;
        break;
      }
      return setvals;
    }

float double_int(int N) 
  {
    int new_N = N * 2;
    //Serial.println('Doubled number: ' + new_N);
    Serial.print(F("Initialising the Bluefruit LE module: "));
    Serial.println(new_N);
    return new_N;
    }


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
