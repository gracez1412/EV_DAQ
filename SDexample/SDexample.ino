/*Using SPI communication

front
USB
1 2 3
4 5 6
back
1-5V
2-MOSI
3-GND
4-MISO
5-SCK
6-RESET

Connections:
GND-GND
VCC-5V
MISO-MISO
MOSI-MOSI
SCK-SCK
CS-D4
*/
// include the SD library:
#include <SPI.h>
#include <SD.h>

//const int chipSelect = 4;
File myFile;

int pinCS = 4;
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  pinMode(pinCS, OUTPUT);

 if(!SD.begin())
 {
  Serial.println("SD card initialization failed :(");
 }

//create and write to file
 myFile = SD.open("test.txt", FILE_WRITE);
 if(myFile)
 {
  Serial.println("Writing to file");
  myFile.println("Testing is correct");
  myFile.close();
  Serial.println("Done writing");
 }
 else
 {
  Serial.println("error opening file");
 }

//open and read file
 myFile = SD.open("test.txt");
 if(myFile)
 {
  Serial.println("Read:");
  while(myFile.available())
  {
    Serial.write(myFile.read());
  }
  myFile.close();
 }
 else
 {
  Serial.println("Failed to open file");
 }

}
 
 
void loop(void)
{
 
}
