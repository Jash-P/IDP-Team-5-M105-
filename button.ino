#include <Adafruit_MotorShield.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);


// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);


int ledPin = 2; // choose the pin for the LED
int inputPin = 3; // Connect sensor to input pin 3






void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");


  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");


  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor1->setSpeed(150);
  myMotor1->run(FORWARD);


  myMotor2->setSpeed(150);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);


  pinMode(ledPin, OUTPUT); // declare LED as output
  pinMode(inputPin, INPUT); // declare pushbutton as input


}


void loop() {
  uint8_t i;


  int val = digitalRead(inputPin); // read input value
  if (val == LOW) { // check if the input is LOW
  digitalWrite(ledPin, LOW); // turn LED OFF


  myMotor1->setSpeed(0); //turn the motors off when the button is not pressed 
  myMotor2->setSpeed(0);
  Serial.print(val);


  } else {


      
      myMotor1->setSpeed(150);
      myMotor1->run(FORWARD);


      myMotor2->setSpeed(150);
      myMotor2->run(FORWARD);


      digitalWrite(ledPin, HIGH); // turn LED ON
      delay(200);
      digitalWrite(ledPin, LOW); // turn LED LOW to flush
      delay(200);
      Serial.print(val);


  }




  
}