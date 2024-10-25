#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;    // variable to store the servo position

#define MAXRANG (520)//the max measurement value of the module is 520cm(a little bit longer
than effective max range)
#define ADCSOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit
// float distT, sensityT
// select the input pin
int sensityPin = A0;
const int e18Sensor = 7;
const int led = 6;

void angleforward(int x){
  for (int i = pos; i <= pos + x; i += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(i);              // tell servo to go to position in variable 'i'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  pos += x; //update the global position value
}

void anglebackward(int x){
  for (int i = pos; i >= pos - x; i -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(i);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  pos -= x; //update the global position value
}


void setup() {
  // Serial init
  Serial.begin(9600);
  pinMode (e18Sensor, INPUT);
  pinMode (led, INPUT);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void grab() {
  angleforward(180);
  delay(3000);
  anglebackward(180);
}

void loop() {
  int state = digitalRead(e18Sensor);
  if(state==HIGH)
  {
  digitalWrite(led, HIGH);
  grab();
 }
else 
digitalWrite(led, LOW);
}