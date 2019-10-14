/*
 Description:

 This sketch shows how to use the SparkFun INA169 Breakout
 Board. As current passes through the shunt resistor (Rs), a
 voltage is generated at the Vout pin. Use an analog read and
 some math to determine the current. The current value is
 displayed through the Serial Monitor.

 Hardware connections:

 Uno Pin    INA169 Board    Function

 +5V        VCC             Power supply
 GND        GND             Ground
 A0         VOUT            Analog voltage measurement

 VIN+ and VIN- need to be connected inline with the positive
 DC power rail of a load (e.g. an Arduino, an LED, etc.).

 */

// Constants
const int SENSOR_PIN = 16;  // Input pin for measuring Vout
const int RS = 0.005;          // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;  // Reference voltage for analog read
const int RL = 51000; //Load resistor value (in ohms)

// Global Variables
float sensorValue;   // Variable to store value from analog read
float current;       // Calculated current value

void setup() {

  // Initialize serial monitor
  Serial.println("setup");
  Serial.begin(9600);
}

void loop() {

  Serial.print("Sensor value: ");
  // Read a value from the INA169 board
  sensorValue = analogRead(SENSOR_PIN);
  
  Serial.println(sensorValue, 10);

  // Remap the ADC value into a voltage number (5V reference)
  sensorValue = (sensorValue * VOLTAGE_REF) / 1023;

  Serial.print("mapped:" );
  Serial.println(sensorValue, 10);

  // Follow the equation given by the INA169 datasheet to
  // determine the current flowing through RS. Assume RL = 10k
  // Is = (Vout x 1k) / (RS x RL)
  current = sensorValue / (RL * RS);

  // Output value (in amps) to the serial monitor to 3 decimal
  // places
  Serial.print(current, 10);
  Serial.println(" A");

  // Delay program for a few milliseconds
  delay(500);

}
