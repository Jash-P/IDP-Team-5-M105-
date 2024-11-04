
#include <Adafruit_MotorShield.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);
#define MAX_RANG   (520)


#define ADC_SOLUTION  (1023.0)


int ledPinR = 2;
int ledPinG = 12; // choose the pin for the LED
int MagnetPin = 11; // pin for the magnetic sensor
int DistancePin = 1; // pin for the distance sensor 








void setup() {




  //LEDs and sensors setup start
  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinG, OUTPUT); // declare LED as output
  pinMode(MagnetPin, INPUT); // declare magnetic sensor as input
  //LEDs and sensors setup end


  //Serial init
  Serial.begin(9600); // sets the bit rate to 9600 per second


}


float dist_t, sensity_t;


void loop() {
  sensity_t = analogRead(DistancePin);


  //dictates when to turn the LED on 
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;
  if (dist_t < 3){
    digitalWrite(ledPinG, HIGH);
  } else
      digitalWrite(ledPinG, LOW);


  uint8_t i;


  int val = digitalRead(MagnetPin); // read input value
  if (val == LOW) { // check if the input is LOW
  digitalWrite(ledPinR, LOW); // turn LED OFF


  
  } else    
      digitalWrite(ledPinR, HIGH); // turn LED ON




  
}