#include<math.h>
int tempPin = 14;    //14 is A0
int hallPin = 15;    //15 is A1
double rawTemp = 0;
double hallValue = 0;

void setup()
{
  Serial.begin(9600);
  //analogReference(INTERNAL); //Putting everything to reference
}

double loop()
{
  rawTemp = analogRead(tempPin);
  //conversion equation
  rawTemp = rawTemp + 709;
  Serial.print("Raw Temp ");
  Serial.println(rawTemp);
  double realTemp = ((5.506 - sqrt(pow(5.506,2) + 4*0.00176 * (870.6 - rawTemp)))/(2 * -0.00176)) + 30;
  Serial.print("Temp ");
  Serial.println(realTemp);
  return realTemp;
  delay(1000);
  
}
