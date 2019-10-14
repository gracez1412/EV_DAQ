int hallPin = 17;    //A0 is 14
double hallValue = 0;
double radius = 0;
double threshold = 330; 
unsigned long timenow = 0;
unsigned long timepast = 0;
unsigned long deltatime = 0; 
double rpm = 0;


void setup() 
{
  Serial.begin(9600);
  analogReference(DEFAULT);
  analogRead(0);
}

void loop()
{
  hallValue = analogRead(hallPin);
  Serial.println(hallValue);

  /*// if hall effect sensor detects magnet 
  if (hallValue < threshold)
  {
    timenow = micros();
    Serial.print("time now: ");
    Serial.println(timenow/1000000.0);
    Serial.print("time past: ");
    Serial.println(timepast/100000.0);
    deltatime = (timenow - timepast)/1000000.0;
    rpm = 1.0/deltatime * 60;
    Serial.print("delta time: ");
    Serial.println(deltatime, 10);
    Serial.print("rpm: ");
    Serial.println(rpm, 10);
    //track how much time has elapsed

    timepast = timenow; //timepast is set to the previous time in which the hall effect sensor encountered the magnet
    while (analogRead(hallPin) < 520)
    {
      //don't do anything
    }

  }*/
    
      //calculate the rpm
    //rpm = (2 * 3.141592 * radius)/deltatime * 1000 * 60; /* [m], [ms], [s], [min] */
    //
    //reset the time 
     
}

/*
Editor's Note:
  The delay time should sync up with the amount of time it takes for the hall effect sensor to pass the magnet (NOT TOO SMALL OR TOO LARGE).
  The threshold value must be a reasonable amount. 
*/

/* ARDUINO CODE IF ABOVE CODE FAILS... 

https://github.com/animeshshastry/motor-rpm-measurement-using-hall-effect-sensor

https://playground.arduino.cc/Main/ReadingRPM

http://engineerexperiences.com/tachometer-using-arduino.html

//this code measures the difference between two rising edges of the digitalised signal coming from hall sensor and then prints the rpm.
 //pin A0 is the signal pin (14)
 int refsig = 200; //for converting the analog signal coming from hall sensor to digital through arduino code
 int val;//the digital value of the incoming analog signals
 int prev_val = 0;
 int t, cur_t; //time variables
 void setup()
 {
   Serial.begin(115200);
   pinMode(A0, INPUT);
 }
 void loop()//Measure RPM
 {
   int sig = analogRead(A0); //read raw value of hall sensor
   if (sig > refsig) val = HIGH; //convert it to digital 0,1 form
   else val = LOW;
   if (prev_val == 0 && val == 1) { //check for rising edge
     cur_t = micros(); //micros returns the current number of millisenconds since beginning this current program 
     Serial.println(1000000 * 60 / (cur_t - t)); //print the rpm
     t = micros(); (essentially same as previos time);
   }
   prev_val = val;
 }


 
*/
