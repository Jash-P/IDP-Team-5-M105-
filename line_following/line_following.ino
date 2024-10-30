#include <Adafruit_MotorShield.h>
#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;    // variable to store the servo position

#define MAXRANG (520)//the max measurement value of the module is 520cm(a little bit longer
than effective max range)
#define ADCSOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit
// float distT, sensityT
// select the input pin
const int e18Sensor = 7;


int sr = 6;   // sensor right
int sl = 5;   // sensor left
int fsr = 8;   // front sensor right
int fsl = 9;   // front sensor left
int fsf = 10; // front sensor forward
int sr_val = 0;  // value for right sensor
int sl_val = 0;  // value for left sensor
int fsr_val = 0;   // value for front sensor right
int fsl_val = 0;   // value for front sensor left
int fsf_val = 0; // value for front sensor forward
int enr = 3;  // enable pin for right motor (PWM)
int enl = 5;  // enable pin for left motor (PWM)
int vspeed = 100;  // variable speed for forward motion
int tspeed = 255;  // turning speed
int tdelay = 20;   // delay during turns
int current_facing = 1;    //1 - north   2 - west   3 - south   4 - east
int last_node_number = 0;
int proximity_state = 0;

void forward()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

// infinitesimal swing turn to the right
void right()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(RELEASE);
  delay(tdelay);
}

// infinitesimal point turn to the right
void pointRight()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  delay(tdelay);
}

// infinitesimal swing turn to the left
void left()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(RELEASE);
  rightMotor->run(FORWARD);
  delay(tdelay);
}

// infinitesimal point turn to the left
void pointLeft()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  delay(tdelay);
}

void stop()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void line_following() //does not detect the junctions/turns!!!
{
    if (sl_val == LOW && sr_val == LOW)
        {  // Both sensors off the line (on the background)
          forward();  // Move forward
        }
        else if (sl_val == HIGH && sr_val == LOW)
        {  // Left sensor on the line, right sensor off the line
          right();  // Turn right to move off the line
        }
        else if (sl_val == LOW && sr_val == HIGH)
        {  // Right sensor on the line, left sensor off the line
          left();  // Turn left to move off the line
        }
        else if (sl_val == HIGH && sr_val == HIGH)
        {  // Both sensors on the line
          stop();  // Stop or make adjustments
        }
}

// pull up when a junction is detected so that back sensors align with the turning point, only for point turn
void pull_up() //pull up for a forward turn
{
  int junctionType; // 1 - forward and right, 2 - forward and left, 3 - right and left, 4 - all sides, 5 - single turn right, 6 - single turn left
    if (fsr_val == HIGH, fsf_val == HIGH, fsl_val == LOW)
      {junctionType = 1;}
    else if (fsr_val == LOW, fsf_val == HIGH, fsl_val == HIGH)
      {junctionType = 2;}
    else if (fsr_val == HIGH, fsf_val == LOW, fsl_val == HIGH)
      {junctionType = 3;}
    else if (fsf_val == HIGH, fsr_val == HIGH, fsl_val == HIGH)
      {junctionType = 4;}
    else if (fsf_val == LOW, fsr_val == HIGH, fsl_val == LOW)
      {junctionType = 5;}
    else if (fsf_val == LOW, fsl_val == HIGH, fsr_val == LOW)
      {junctionType = 6;}

  switch (junctionType)
  {
    case 1:
      while (!(sl_val == HIGH && sr_val == HIGH)) //move until both of them are on line
      {
        line_following;
      }
      while (fsf_val == LOW)
      {
        left(); //finish this pull-up in the position forward as well for a more convenient use at the high/level
      }

    case 2:
      while (!(sl_val == HIGH && sr_val == HIGH)) //move until both of them are on line
      {
        line_following;
      }
      while (fsf_val == LOW)
      {
        right();
      }

    case 3:
      while (!(sl_val == HIGH && sr_val == HIGH)) //move until both of them are on line
      {
        line_following;
      }
    
    case 4:
      while (!(sl_val == HIGH && sr_val == HIGH))
      {
        line_following;
      }

    /* case 5:
      while (!(sl_val == HIGH && sr_val == HIGH)) //move until both of them are on line
      {
        line_following;
      }
      while (fsf_val == LOW)
      {
        left(); //finish this pull-up in the position forward as well for a more convenient use at the high/level
      }
    
    case 6:
      while (!(sl_val == HIGH && sr_val == HIGH)) //move until both of them are on line
      {
        line_following;
      }
      while (fsf_val == LOW)
      {
        right();
      }
    */

  }

}

// This function is for swing turn. !!!A point to consider: we still need pull up for swing turn
void swingTurnRight() {

  pull_up();

  // Ignore the initial high reading (sensor on the line)
  while (digitalRead(fsf) == HIGH) {
    // Wait until sensor no longer detects the line (goes low)
	right();
  }

  // Turn right until the sensor detects a line again (becomes high)
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	right();
  }

  if (digitalRead(fsf) == HIGH)
  {
  // Stop turning when another line is found
  stop();
  }

  // update facing
  current_facing = current_facing - 1;
  if (current_facing == 0)
  {
    current_facing = 4;
  }
}

void pointTurnRight() {
  pull_up();

  // Ignore the initial high reading (sensor on the line)
  while (digitalRead(fsf) == HIGH) {
    // Wait until sensor no longer detects the line (goes low)
	pointRight();
  }

  // Turn right until the sensor detects a line again (becomes high)
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	pointRight();
  }

  if (digitalRead(fsf) == HIGH)
  {
  // Stop turning when another line is found
  stop();
  }

  // update facing
  current_facing = current_facing - 1;
  if (current_facing == 0)
  {
    current_facing = 4;
  }
}

void swingTurnLeft(){
  // pull_up();

  // Ignore the initial high reading (sensor on the line)
  while (digitalRead(fsf) == HIGH) {
  // Wait until sensor no longer detects the line (goes low)
	left();
  }

  // Turn right until the sensor detects a line again (becomes high)
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	left();
  }

  if (digitalRead(fsf) == HIGH)
  {
  // Stop turning when another line is found
  	stop();
  }
  
  // update the facing
  current_facing = current_facing + 1;
    if (current_facing == 5)
    {
      current_facing = 1;
    }
}

void pointTurnLeft(){

  pull_up();

  // Ignore the initial high reading (sensor on the line)
  while (digitalRead(fsf) == HIGH) {
    // Wait until sensor no longer detects the line (goes low)
	pointLeft();
  }

  // Turn right until the sensor detects a line again (becomes high)
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	pointLeft();
  }

  if (digitalRead(fsf) == HIGH)
  {
  // Stop turning when another line is found
  	stop();
  }
  
  // update the facing
  current_facing = current_facing + 1;
    if (current_facing == 5)
    {
      current_facing = 1;
    }
}

void 180_turn(){

  // Ignore the initial high reading (sensor on the line)
  while (digitalRead(fsf) == HIGH) {
    // Wait until sensor no longer detects the line (goes low)
	pointLeft();
  }

  // Turn right until the sensor detects a line again (becomes high)
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	pointLeft();
  }

  if (digitalRead(fsf) == HIGH)
  {
  // Stop turning when another line is found
  	stop();
  }
  
  // update the facing
  current_facing = current_facing + 2;
    if (current_facing == 6)
    {
      current_facing = 2;
    }
    else if (current_facing == 5)
    {
      current_facing = 1;
    }
}

void go_forward() //going straight untill the next single turn/junction
{
  forward();
  delay(10);
  while (/*fsf_val == HIGH && */fsr_val == LOW && fsl_val == LOW) // no junction or single turn detected so go straight 
  {
    line_following();
  } 
}

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

// detect the boxes and grab
void grab() {
  int i = 0;
  while (proximity_state == LOW && i <= 2000)
  {
    forward();
    i++;
  }

  if(proximity_state==HIGH)
  {
  angleforward(180);
  delay(3000);
  anglebackward(180);
  }
}

void route_to_factory() //hardcoded route to the factory (just gets there)
{
  go_forward();
  
  if (fsf_val == HIGH, fsr == HIGH, fsl == HIGH) //confirm that we are at junction 1
  {
    last_node_number = 1;
    go_forward();
  } 

  if (fsf_val == HIGH, fsr == HIGH, fsl == HIGH) //confirm that we are at junction 2
  {
    last_node_number = 2;
    go_forward();
  } 

  if (fsf_val == LOW, fsr == HIGH, fsl == HIGH) //confirm that we are at junction 3
  {
    last_node_number = 3;
    pointTurnRight();
    go_forward();
  } 

  if (fsf_val == HIGH, fsr == HIGH, fsl == HIGH) //confirm that we are at junction 4
  {
    last_node_number = 4;
    go_forward();
  } 

  if (fsf_val == HIGH, fsr == HIGH, fsl == HIGH) //confirm that we are at junction 5
  {
    last_node_number = 5;
    go_forward();
  } 

  if (fsf_val == HIGH, fsr == HIGH, fsl == HIGH) //confirm that we are at junction 6
  {
    last_node_number = 6;
    go_forward();
  } 

  if (fsf_val == LOW, fsr == HIGH, fsl == HIGH) //confirm that we are at junction 7
  {
    last_node_number = 7;
    pointTurnLeft();
    go_forward();
  } 

  if (fsf_val == HIGH, fsr == LOW, fsl == HIGH) //confirm that we are at junction 9
  {
    last_node_number = 9;
    go_forward();
  } 

  if (fsf_val == LOW, fsr == LOW, fsl == HIGH) //confirm that we are at the single turn after junction 9
  {
    pointTurnLeft();
    go_forward();
  } 

  if (fsf_val == HIGH, fsr == LOW, fsl == HIGH) //confirm that we are at junction 20
  {
    last_node_number = 20;
    pointTurnLeft();
    // try grabbing four times
    grab_boxes();
    grab_boxes();
    grab_boxes();
    grab_boxes();
  } 
}


void setup()
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  // set up for the grabbing mechanism
  pinMode (e18Sensor, INPUT);
  pinMode (led, INPUT);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  if (!AFMS.begin())
  {         // create with the default frequency 1.6KHz
    // OR with a different frequency, say 1KHz
    // if (!AFMS.begin(1000))
      Serial.println("Could not find Motor Shield. Check wiring.");
      while (1);
  }
  Serial.println("Motor Shield found.");
  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  pinMode(sr, INPUT);
  pinMode(sl, INPUT);
  pinMode(fsr, INPUT);
  pinMode(fsl, INPUT);
  pinMode(fsf, INPUT);
  pinMode(11, OUTPUT);
  delay(2000);  // delay for system start-up
}

// testing: should be able to follow the line, turn at a single turn where there's no junction and finally stop at a junction
void loop()
{
  sr_val = digitalRead(sr);  // read right sensor
  sl_val = digitalRead(sl);  // read left sensor
  fsr_val = digitalRead(fsr);   // value for front sensor right
  fsl_val = digitalRead(fsl);   // value for front sensor left
  fsf_val = digitalRead(fsf); // value for front sensor forward
  proximity_state = digitalRead(e18Sensor);

  digitalWrite(11, LOW);
  // go to factory to pick up boxes
  route_to_factory();
  //make a 180 degrees turn
  180_turn();
  /* if (/*fsf_val == HIGH && */fsr_val == LOW && fsl_val == LOW) // no junction or single turn detected so go straight 
  {
    line_following();
  } 
  else if (fsf_val == LOW && fsr_val == HIGH && fsl_val == LOW)  // single right turn
  {
    swingTurnRight();

  }
  else if (fsf_val == LOW && fsr_val == LOW && fsl_val == HIGH)  // single left turn
  {
    swingTurnLeft();
    
  }
  else                                        // junction detected
  {
    stop();
  }
  {
    digitalWrite(11, HIGH);
  }*/
  // delay(20);
}