

void setup() {
  Serial1.begin(9600);   // Communication with HM-10
}

void loop() {

  if (Serial1.available()) {
      char data = Serial1.read();    // Read the data from the Bluetooth module
      Serial.write(data);            // Send the data to the computer (laptop)
    }



    // Display elapsed time
    Serial.print(totalTimeinSeconds, 3);
    Serial.print(", ");

    Serial.print(weight0);
    Serial.print(", ");

    Serial.print(weight1);
    Serial.print(", ");

    Serial.println(weight2);

}
