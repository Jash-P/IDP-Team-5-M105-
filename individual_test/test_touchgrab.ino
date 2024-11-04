#include <Servo.h>
Servo myservo1;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
// float distT, sensityT
// select the input pin
const int led = 12;
int crashswitchPin = 11;
int crash_state = HIGH;

void angleforward(int x)
{ // turns the angle of the servo forward by x degrees
  for (int i = pos; i <= pos + x; i += 1) 
  { // goes from 0 degrees to x degrees
    // in steps of 1 degree
    myservo1.write(i);              // tell servo to go to position in variable 'i'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  pos += x; //update the global position value
}

void anglebackward(int x)
{ // turns the angle of the servo backwards by x degrees
  for (int i = pos; i >= pos - x; i -= 1) 
  { // goes from 0 degrees to 180 degrees in steps of 1 degree
    myservo1.write(i);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  pos -= x; //update the global position value
}

void setup() 
{
  // Serial init
  Serial.begin(9600);
  pinMode (led, OUTPUT);
  myservo1.attach(13); // attaches the servo on pin 9 to the servo object
  pinMode(crashswitchPin, INPUT); //sets the crashswitch as input
}

void grab() 
{ // grab function to be called when grabber system needs to be operated
  angleforward(180); // calling rotation functions
  delay(3000);
  anglebackward(180);
}

void loop() 
{
  crash_state = digitalRead(crashswitchPin); // reads the input value of the crash switch
  if(crash_state==LOW)
  {
    digitalWrite(led, HIGH); // if switch is pressed, light pick-up LED and operate grab function
    grab();
  }
  else
  {
    digitalWrite(led, LOW);  // otherwise LED is off and grabber is still
  }
}