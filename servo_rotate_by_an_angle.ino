#include <Servo.h>
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;    // variable to store the servo position
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
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}
void loop() {
  angleforward(120);
  delay(1000);
  anglebackward(120);
  delay(1000);
  //anglebackward(30);
}