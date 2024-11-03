#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);
// Select M3 port, as 1 and 2 are used for wheels
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);
int inputPin = 7; // Connect button to input pin 3
void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor3->setSpeed(150);
  myMotor3->run(FORWARD);
  // turn on motor
  myMotor3->run(RELEASE);
  pinMode(inputPin, INPUT); // declare pushbutton as input, this input will later be from code saying it has arrived at the drop off point
}
void loop() {
  uint8_t i;
  int val = digitalRead(inputPin); // read input value
  if (val == LOW) { // check if the input is LOW
  myMotor3->setSpeed(0); //turn the motors off when the button is not pressed
  Serial.print(val);
  } else {
      myMotor3->setSpeed(150);
      myMotor3->run(FORWARD);
      delay(4000); // this will be the time the servo turns for, will need to adjust later for release mechanism
      myMotor3->setSpeed(0);
  }
}
