#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11);

void setup() {
  Serial.begin(9600);                 // Serial port to computer
  HC12.begin(9600);                   // Serial port to HC12
}

void loop() {

  while (HC12.available()) {        // If HC-12 has data
    Serial.write(HC12.read());      // Send the data to Serial monitor
  }
}
