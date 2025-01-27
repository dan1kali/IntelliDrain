#include <Wire.h>
#include <LiquidCrystal.h>    // Use this for parallel connection LCD



// hello


// Settings
#define BTN_PIN             9                         // Pin that connects to the button (Pin 7 here)
#define SAMPLING_FREQ_HZ    4                         // Sampling frequency (Hz)
#define SAMPLING_PERIOD_MS  1000 / SAMPLING_FREQ_HZ   // Sampling period (ms)
#define NUM_SAMPLES         8                         // 8 samples at 4 Hz is 2 seconds
#define DEBOUNCE_DELAY      30                        // Delay for debounce (ms)
#define SAMPLE_DELAY        500                       // Delay between samples (ms)

// Used to remember what mode we're in
#define NUM_MODES           2
#define MODE_IDLE           0
#define MODE_RECORDING      1

LiquidCrystal lcd(12, 11, 5, 6, 7, 8);  // LCD connection (RS, EN, D4, D5, D6, D7)

const int Output_Pin = 18;  // Pin 18 on Arduino Mega is interrupt-capable
volatile int  Pulse_Count;
unsigned int  Milliliters_per_minute;
unsigned long Current_Time, Loop_Time, Start_Time, timestamp;

void setup() {
  // Initialize button
  pinMode(BTN_PIN, INPUT_PULLUP);  // Use internal pull-up resistor for the button
  pinMode(Output_Pin, INPUT);

  // Start serial
  Serial.begin(115200);
  Serial.println("Serial Initialized");  // Debug message to check if setup is working

  // Initialize the LCD
  lcd.begin(16, 2);  // 16 columns, 2 rows
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IDLE");

  // Attach interrupt for Pin 18
  attachInterrupt(digitalPinToInterrupt(18), Detect_Rising_Edge, RISING);  // Using Pin 18 for interrupt
}

void loop() {
  static uint8_t mode = MODE_IDLE;
  static uint8_t btn_state;
  static uint8_t last_btn_state = HIGH;
  static unsigned long last_dbnc_time = 0;
  int btn_reading;

  static unsigned long sample_timestamp = millis();
  static unsigned long start_timestamp = millis();

  // Debounce button - see if button state has changed
  btn_reading = digitalRead(BTN_PIN);
 // Serial.print("Button reading: ");   // Debugging button state
 //  Serial.println(btn_reading);        // Show the raw button state (HIGH or LOW)

  if (btn_reading != last_btn_state) {
    last_dbnc_time = millis();
  }

  // Debounce button - wait some time before checking the button again
  if ((millis() - last_dbnc_time) > DEBOUNCE_DELAY) {
    if (btn_reading != btn_state) {
      btn_state = btn_reading;
      
      // Only transition to new mode if button is pressed (LOW)
      if (btn_state == LOW) {
        mode = (mode + 1);
        if (mode >= NUM_MODES) {
          mode = MODE_IDLE;
        }

        // Debugging mode switch
        Serial.print("Mode switched to: ");
        Serial.println(mode == MODE_IDLE ? "IDLE" : "Recording");

        // Update LCD based on the mode
        lcd.clear();
        switch (mode) {
          case MODE_IDLE:
            lcd.setCursor(0, 0);
            lcd.print("IDLE");
            Serial.println("Switched to IDLE mode");
            break;
          case MODE_RECORDING:
            lcd.setCursor(0, 0);
            lcd.print("Recording");
            Serial.println("Recording mode activated");
            start_timestamp = millis();
            break;
          default:
            break;
        }
      }
    }
  }

  // Debounce button - update the last button state
  last_btn_state = btn_reading;

  // Only collect flow data if not in IDLE mode
  if (mode > MODE_IDLE) {
    if ((millis() - sample_timestamp) >= SAMPLE_DELAY) {
      sample_timestamp = millis();
      Current_Time = millis();
      if (Current_Time >= (Loop_Time + 1000)) {
        Loop_Time = Current_Time;
        
        // Flow rate calculation and debug print (mL/min)
        Milliliters_per_minute = (Pulse_Count * 1000 / 7.5);  // Convert to mL/min
        Serial.print("Pulse Count: ");
        Serial.print(Pulse_Count);   // Debugging pulse count
        Serial.print(", mL/min: ");
        Serial.println(Milliliters_per_minute);  // Print flow rate in mL/min

        // Update LCD with flow rate
        lcd.setCursor(0, 0);
        lcd.print("Flow Rate:");
        lcd.setCursor(0, 1);
        lcd.print("              "); // Clear the old flow rate
        lcd.print(Milliliters_per_minute);
        lcd.print(" mL/min");

        // Reset pulse count every second
        Pulse_Count = 0;
      }
    }
  }
}

void Detect_Rising_Edge () {
  Pulse_Count++;  // Increment pulse count on each rising edge
  Serial.print("Pulse detected: ");
  Serial.println(Pulse_Count);  // Print each pulse count for debugging
}
