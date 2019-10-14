//----------------------------------------------
//
//        Sketch Generated by Visuino
//              www.visuino.com
//          Version 7.8.2.290
//
//------------------ Source --------------------
//
// Setup.visuino
//
//----------------------------------------------

#define VISUINO_ARDUINO_NANO

#include <OpenWire.h>
#include <Mitov.h>
#include <Mitov_FormattedSerial.h>
#include <Mitov_LiquidCrystalDisplay.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(2, 3, 4, 5,6,7);

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

//The setup function is called once at startup of the sketch
void setup()
{
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
  delay(5000);
}

int number = 0;
int bat = 5;
int temp = 70;

void loop()
{
  unsigned long A_Current_Microseconds = micros();

  BoardDeclarations::SerialOutput0.SystemLoopBegin( A_Current_Microseconds );
  BoardDeclarations::SerialPort0.SystemLoopBegin( A_Current_Microseconds );

  lcd.print("Speed: ");
  lcd.print(number);
  lcd.print(" m/s");
  lcd.setCursor(0,1);
  lcd.print("Battery: ");
  lcd.print(bat);
  lcd.print(" V");
  lcd.setCursor(0,0);
  number++;
  delay(1500); // wait 1 second
  lcd.clear();

  lcd.print("Temperature: ");
  lcd.print(temp);
  lcd.print(" F");
  delay(1500); // wait 1 second
  lcd.clear();
}