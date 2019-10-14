
#include<math.h>

#define VISUINO_ARDUINO_NANO
#include <OpenWire.h>
#include <Mitov.h>
#include <Mitov_FormattedSerial.h>
#include <Mitov_LiquidCrystalDisplay.h>
#include <LiquidCrystal.h>

#include <SPI.h>
#include <SD.h>

#include <SoftwareSerial.h>


//////////////////////////////////////////
//// Pin Definitions
//////////////////////////////
const int TEMP_PIN1   = 14;    //14 is A0
const int TEMP_PIN2   = 15;    //15 is A1
const int TEMP_PIN3   = 16;    //16 is A2
const int HALL_PIN    = 17;    //17 is A3
const int SENSOR_PIN  = 18;    //18 is A4
LiquidCrystal lcd(2, 3, 4, 5,6,7); //Digital 2,3,4,5,6,7
const int PIN_CS      = 8;       //4 is D4

//////////////////////////////////////////
//// Variables
//////////////////////////////

// Temperature Sensor

// Hall Effect Sensor 
double hallValue = 0;
double radius = .254; //.3 meters 
double threshold = 470; //previously 480 
unsigned long timenow = 0;
unsigned long timepast = 0;
double deltatime = 0; 
double rpm = 0;

// Current Sensor
const int RS = 0.02;          // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;  // Reference voltage for analog read
const int RL = 10000; //Load resistor value (in ohms)
float sensorValue;   // Variable to store value from analog read
float current;       // Calculated current value

// SD Card
File myFile;

// Wireless
SoftwareSerial HC12(10, 11);

//////////////////////////////////////////
//// Functions 
//////////////////////////////

// Temperature Sensors
double runTemperature (const int pin)
{
  Serial.println(micros());
  double rawTemp = analogRead(pin);
  
  //conversion equation
  rawTemp = rawTemp + 709;
  double realTemp = ((5.506 - sqrt(pow(5.506,2) + 4*0.00176 * (870.6 - rawTemp)))/(2 * -0.00176)) + 27;
  //Serial.print("Temp");
  //Serial.print(pin);
  //Serial.print(": ");
  //Serial.println(realTemp);
  return realTemp;
}

// Hall Effect Sensor
double runHallEffect () 
{
   hallValue = analogRead(HALL_PIN);
   /*while(true){
    Serial.println(analogRead(HALL_PIN));
   }*/
  
  //Serial.println("runHallEffect is running");
  Serial.println(analogRead(HALL_PIN));
  // if hall effect sensor detects magnet 
  if (hallValue < 490)
  {
    timenow = micros();
    Serial.print("Detect below threshold");
    //Serial.print("time now: ");
    //Serial.println(timenow/1000000.0);
    //Serial.print("time past: ");
    //Serial.println(timepast/100000.0);
    deltatime = (timenow - timepast)/1000000.0;
    //rpm = 1.0/deltatime * 60;
    //Serial.print("delta time: ");
    //Serial.println(deltatime, 10);
    Serial.print("rpm: ");
    Serial.println(rpm, 10);
    //track how much time has elapsed
     /*the most current time*/
    timepast = timenow; //timepast is set to the previous time in which the hall effect sensor encountered the magnet
    while (analogRead(HALL_PIN) < 500) //previously 430
    {
      //don't do anything
      Serial.println("in while");
    }

  }
    Serial.println("dt");
    Serial.println(deltatime, 10);
      //calculate the rpm
    rpm = (2 * 3.141592 * radius)/(deltatime); /* [m], [ms], [s], [min] */
    Serial.println(rpm);
    //
    //reset the time 
     return rpm;
} 

double runCurrent()
{
  //Serial.println(micros());
  //Serial.println("Current sensor is running");
  // Read a value from the INA169 board
  sensorValue = analogRead(SENSOR_PIN);
  
  // Remap the ADC value into a voltage number (5V reference)
  sensorValue = (sensorValue * VOLTAGE_REF) / 1023;

  // Follow the equation given by the INA169 datasheet to
  // determine the current flowing through RS. Assume RL = 10k
  // Is = (Vout x 1k) / (RS x RL)
  current = sensorValue / (RL * RS);

  return current;
} 

//////////////////////////////////////////
//// LCD Display
//////////////////////////////

// Arduino Board Declarations
namespace BoardDeclarations
{
Mitov::TypedSerialPort<Mitov::SerialPort<SERIAL_TYPE, Serial>, SERIAL_TYPE, Serial> SerialPort0;
Mitov::ArduinoSerialOutput<Mitov::TypedSerialPort<Mitov::SerialPort<SERIAL_TYPE, Serial>, SERIAL_TYPE, Serial>, SerialPort0> SerialOutput0;
} // BoardDeclarations

// Declarations
namespace Declarations
{
Mitov::LiquidCrystalDisplay<Mitov::LiquidCrystalDisplayParallel<4>> LiquidCrystalDisplay1 = Mitov::LiquidCrystalDisplay<Mitov::LiquidCrystalDisplayParallel<4>>( 16, 2 );
Mitov::LiquidCrystalDisplayCharInput<Mitov::LiquidCrystalDisplay<Mitov::LiquidCrystalDisplayParallel<4>>, LiquidCrystalDisplay1> LiquidCrystalDisplay1_InputChar_1;
} // Declarations

// Type Converters
namespace TypeConverters
{
Mitov::Convert_BinaryBlockToChar Converter0;
} // TypeConverters

// Pin Call Declarations
namespace PinCalls
{
void PinCallerReceive1( void *_Data );
void PinCallerReceive2( void *_Data );
} // PinCalls

// Pin Call Implementations
namespace PinCalls
{
void PinCallerReceive2( void *_Data )
{
  TypeConverters::Converter0.Convert( _Data, PinCallerReceive1 );
}

void PinCallerReceive1( void *_Data )
{
  Declarations::LiquidCrystalDisplay1_InputChar_1.InputPin_o_Receive( _Data );
}
} // PinCalls

namespace ComponentsHardware
{
void SystemUpdateHardware()
{
}
} // ComponentsHardware

void setup() {
  // put your setup code here, to run once:
  Serial.println(micros());
  Serial.begin(9600);
  analogReference(DEFAULT);
  analogRead(0);

  // SD Card File 
  pinMode (PIN_CS, OUTPUT);
  myFile = SD.open("DAQ.txt", FILE_WRITE);
  if(!SD.begin())
  {
    Serial.println("SD card initialization failed :(");
  }
  if (myFile)
  {
    Serial.println("SD card setup");
    myFile.println("date,time,speed,temp1,temp2,temp3,current");
    myFile.close();
  }
  
   HC12.begin(9600);                   // Serial port to HC12

  // LCD Display
  {
    static uint8_t _Pins[] = { 2, 255, 3, 4, 5, 6, 7 };
    Declarations::LiquidCrystalDisplay1.__Pins = _Pins;
  }

  BoardDeclarations::SerialOutput0.OutputPin().SetCallback( PinCalls::PinCallerReceive2 );

  BoardDeclarations::SerialPort0.SystemInit();
  Declarations::LiquidCrystalDisplay1.SystemInit();

  OpenWire::SystemStarted();

  lcd.begin(16, 2);
  Serial.begin(9600);
  lcd.print("Welcome to SMV");
  lcd.setCursor(0,1);
  lcd.print("Driver");
  lcd.setCursor(0,0);
  delay(2000);
}

void writetoLCD (double speed, double current)
{
  Serial.println(micros());
   Serial.println("write to LCD is running");
  unsigned long A_Current_Microseconds = micros();

  BoardDeclarations::SerialOutput0.SystemLoopBegin( A_Current_Microseconds );
  BoardDeclarations::SerialPort0.SystemLoopBegin( A_Current_Microseconds );
  
  //Serial.println("print speed");
  lcd.clear(); 
  lcd.print("Speed: ");
  lcd.print(speed);
  lcd.print(" m/s");
  lcd.setCursor(0,1);
  
  lcd.print("Joules: ");
  lcd.print(current);
  lcd.print(" J");
  lcd.setCursor(0,0);
  //delay(200); // wait 1.5 second 
  
}

/*void writeToLCD ()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("works");
}*/

void writeToSD(double speed, double temp1, double temp2, double temp3, double current)
{
  myFile = SD.open("DAQ.txt",FILE_WRITE);
  if (!SD.begin())
      {
        Serial.println("SD card initialization failed :(");  
      }
  if (myFile) {
    unsigned long time = micros();
    myFile.print(time/1000000.0, 6); //prints s from time run 
    myFile.print(","); 
    myFile.print(speed); //speed
    myFile.print(","); 
    myFile.print(temp1); 
    myFile.print(","); 
    myFile.print(temp2); 
    myFile.print(","); 
    myFile.print(temp3); 
    myFile.print(","); 
    myFile.print(current); //speed
    myFile.println();
    myFile.close();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(micros());
  double HS = runHallEffect();
  double t1 = runTemperature (TEMP_PIN1);
  double t2 = runTemperature (TEMP_PIN2);
  double t3 = runTemperature (TEMP_PIN3);
  double c = runCurrent();
  writetoLCD (HS, c);// run temperature sensor 
  //writeToSD(HS, t1, t2, t3, c);
  Serial.println("finsihed the wrtie to CLD");

  while (HC12.available()) {        // If HC-12 has data
    Serial.write(HC12.read());      // Send the data to Serial monitor
  }
}
