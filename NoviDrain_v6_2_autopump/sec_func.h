#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_) && not defined(__SAMD51__)
  #include <SoftwareSerial.h>
#endif


#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#define B1SELECT                        100   // Absess rate - Low  V0-255   Button 1 selection
#define B2SELECT                        200   // Absess rate - High V0-255   Button 2 selection
#define B3SELECT                        100   // Saline rate - Low  V0-255   Button 3 selection
#define B4SELECT                        200   // Saline rate - High V0-255   Button 4 selection
#define B5SELECT                        18   // Flush Len - Short  Seconds  Button 5 selection
#define B6SELECT                        18   // Flush Len - Long   Seconds  Button 6 selection
#define B7SELECT                        2   // Flush Freq-Auto    Minutes  Button 7 selection
#define B8SELECT                        -1   // Flush NOW                   Button 8 selection
#define B9SELECT                        220   // Button 9 selection

void setup_ble(Adafruit_BluefruitLE_UART *ble);
Adafruit_BluefruitLE_UART connect_ble(Adafruit_BluefruitLE_UART ble);
int check_ble_data(Adafruit_BluefruitLE_UART *ble);
void update_settings(int b, int setvals[]);
void update_settingsx(int setvals[]);


float double_int(int N);
  
