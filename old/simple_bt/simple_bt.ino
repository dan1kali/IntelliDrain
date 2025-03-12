#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3);
int ledPin = 13;

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  int i;

  if (mySerial.available()) {
    i = mySerial.read();
    Serial.println("DATA RECEIVED:");

    if (i == '1') {
      digitalWrite(ledPin, HIGH);
      Serial.println("LED ON");
    }

    if (i == '0') {
      digitalWrite(ledPin, LOW);
      Serial.println("LED OFF");
    }
  }
}

