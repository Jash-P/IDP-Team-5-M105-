#include <Adafruit_MotorShield.h>
#include <Servo.h>
// important notice: don't use pin7 because it's broken!!!!!!!!!!!!!!!!!!!!!
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);
Servo myservo1;  // create servo object to control pick up mechanism
Servo myservo2;  // create servo object to control release mechanism
int pos1 = 0;    // variable to store the pick up mechanism angular position
int pos2 = 0;    // variable to store the release mechanism angular position

#define MAXRANG (520)//the max measurement value of the module is 520cm(a little bit longer than effective max range)
#define ADCSOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit
// float distT, sensityT
// select the input pin
const int e18Sensor = 7;

int sr = 6;   // sensor right
int sl = 5;   // sensor left
int LED = 2;  // LED indicator
int fsr = 8;   // front sensor right
int fsl = 9;   // front sensor left
int fsf = 10; // front sensor forward
int crush = 11; // crush sensor
int sr_val = 0;  // value for right sensor
int sl_val = 0;  // value for left sensor
int fsr_val = 0;   // value for front sensor right
int fsl_val = 0;   // value for front sensor left
int fsf_val = 0; // value for front sensor forward
int enr = 3;  // enable pin for right motor (PWM)
int enl = 5;  // enable pin for left motor (PWM)
int vspeed = 255;  // variable speed for forward motion
int tspeed = 255;  // turning speed
int tdelay = 1;   // delay during turns
int current_facing = 1;    //1 - north   2 - west   3 - south   4 - east
int last_node_number = 0;
int proximity_state = 0;
int crush_state = 0;



void setup()
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  // set up for the grabbing mechanism
  pinMode (e18Sensor, INPUT);
  myservo1.attach(1);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(4);
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
  pinMode(LED, OUTPUT);
  pinMode(sr, INPUT);
  pinMode(sl, INPUT);
  pinMode(fsr, INPUT);
  pinMode(fsl, INPUT);
  pinMode(fsf, INPUT);
  //pinMode(11, OUTPUT);
  pinMode(crush, INPUT);
  delay(2000);  // delay for system start-up
}


void update_values()
{
  sr_val = digitalRead(sr);  // read right sensor
  sl_val = digitalRead(sl);  // read left sensor
  fsr_val = digitalRead(fsr);   // value for front sensor right
  fsl_val = digitalRead(fsl);   // value for front sensor left
  fsf_val = digitalRead(fsf); // value for front sensor forward
  proximity_state = digitalRead(e18Sensor);
  crush_state = digitalRead(crush);
}

void forward()
{
  leftMotor->setSpeed(210);
  rightMotor->setSpeed(210);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void backward()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

// infinitesimal swing turn to the right
void right()
{
  leftMotor->setSpeed(200);
  rightMotor->setSpeed(200);
  leftMotor->run(BACKWARD);
  rightMotor->run(RELEASE);
  delay(tdelay);
}

// infinitesimal point turn to the right
void pointRight()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  delay(tdelay);
}

// infinitesimal swing turn to the left
void left()
{
  leftMotor->setSpeed(200);
  rightMotor->setSpeed(200);
  leftMotor->run(RELEASE);
  rightMotor->run(BACKWARD);
  delay(tdelay);
}

// infinitesimal point turn to the left
void pointLeft()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
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
    update_values();
    if (sl_val == LOW && sr_val == LOW)
        {  // Both sensors off the line (on the background)
          forward();  // Move forward
        }
        else if (sl_val == HIGH && sr_val == LOW)
        {  // Left sensor on the line, right sensor off the line
          left();  // Turn right to move off the line
        }
        else if (sl_val == LOW && sr_val == HIGH)
        {  // Right sensor on the line, left sensor off the line
          right();  // Turn left to move off the line
        }
        else if (sl_val == HIGH && sr_val == HIGH)
          //{  // Both sensors on the line
          forward();  // Stop or make adjustments
          //}
    update_values();
}

void backward_line_following() //does not detect the junctions/turns!!!
{
    update_values();
    if (sl_val == LOW && sr_val == LOW)
        {  // Both sensors off the line (on the background)
          backward();  // Move forward
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
          //{  // Both sensors on the line
          backward();  // Stop or make adjustments
          //}
    update_values();
}

// pull up when a junction is detected so that back sensors align with the turning point, only for point turn
void pull_up() //pull up for a forward turn
{
  int junctionType; // 1 - forward and right, 2 - forward and left, 3 - right and left, 4 - all sides, 5 - single turn right, 6 - single turn left
  update_values();
    if (fsr_val == HIGH && fsf_val == HIGH && fsl_val == LOW)
      {junctionType = 1;}
    else if (fsr_val == LOW && fsf_val == HIGH && fsl_val == HIGH)
      {junctionType = 2;}
    else if (fsr_val == HIGH && fsf_val == LOW && fsl_val == HIGH)
      {junctionType = 3;}
    else if (fsf_val == HIGH && fsr_val == HIGH && fsl_val == HIGH)
      {junctionType = 4;}
    else if (fsf_val == LOW && fsr_val == HIGH && fsl_val == LOW)
      {junctionType = 5;}
    else if (fsf_val == LOW && fsl_val == HIGH && fsr_val == LOW)
      {junctionType = 6;}

  switch (junctionType)
  {
    case 1:
      update_values();
      while (!(sl_val == HIGH && sr_val == HIGH)) //move until both of them are on line
      {
        line_following;
        update_values();
      }
      update_values();
      while (fsf_val == LOW)
      {
        left(); //finish this pull-up in the position forward as well for a more convenient use at the high/level
        update_values();
      }
      break;

    case 2:
      update_values();
      while (!(sl_val == HIGH && sr_val == HIGH)) //move until both of them are on line
      {
        line_following;
        update_values();
      }
      update_values();
      while (fsf_val == LOW)
      {
        right();
        update_values();
      }
      break;
      

    case 3:
    case 4:
      update_values();
      while (!(sl_val == HIGH && sr_val == HIGH))
      {
        line_following;
        update_values();
      }
      break;

    /* We don't really need pull up for single turn because we only have 2 single turns and it's impossible to code unless it's disgusting
    case 5:
      while (!(sl_val == HIGH && sr_val == HIGH)) //move until both of them are on line
      {
        line_following;
      }
      while (fsf_val == LOW)
      {
        left(); //finish this pull-up in the position forward as well for a more convenient use at the high/level
      }
      break;
    
    case 6:
      while (!(sl_val == HIGH && sr_val == HIGH)) //move until both of them are on line
      {
        line_following;
      }
      while (fsf_val == LOW)
      {
        right();
      }
      break;
    */  

  }

}

// This function is for swing turn. !!!A point to consider: we still need pull up for swing turn
void swingTurnRight() {

  // pull_up();

  // Ignore the initial high reading (sensor on the line)
  update_values();
  while (digitalRead(fsf) == HIGH) {
    // Wait until sensor no longer detects the line (goes low)
	right();
  update_values();
  }
  delay(500);
  // Turn right until the sensor detects a line again (becomes high)
  update_values();
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	right();
  update_values();
  }

  update_values();
  if (digitalRead(fsf) == HIGH)
  {
  // Stop turning when another line is found
  stop();
  }
  update_values();

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
  update_values();
  while (digitalRead(fsf) == HIGH) {
    // Wait until sensor no longer detects the line (goes low)
	pointRight();
    update_values();
  }

  // Turn right until the sensor detects a line again (becomes high)
  update_values();
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	pointRight();
    update_values();
  }

  update_values();
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
  update_values();
  while (digitalRead(fsf) == HIGH) {
  // Wait until sensor no longer detects the line (goes low)
	left();
  update_values();
  }
  delay(500);
  // Turn right until the sensor detects a line again (becomes high)
  update_values();
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	left();
  update_values();
  }

  update_values();
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
  update_values();
  while (digitalRead(fsf) == HIGH) {
    // Wait until sensor no longer detects the line (goes low)
	pointLeft();
  update_values();
  }

  // Turn right until the sensor detects a line again (becomes high)
  update_values();
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	pointLeft();
  update_values();
  }

  update_values();
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

void turn_180(){

  // Ignore the initial high reading (sensor on the line)
  update_values();
  while (digitalRead(fsf) == HIGH) {
    // Wait until sensor no longer detects the line (goes low)
	pointRight();
  update_values();
  }

  // Turn right until the sensor detects a line again (becomes high)
  update_values();
  while (digitalRead(fsf) == LOW) {
    // Keep turning right until a line is detected
	pointRight();
  update_values();
  }

  update_values();
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

void drop_at_red()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  update_values();
  while (fsr_val == HIGH && fsl_val == HIGH)
  {
    delay(5);
    update_values();
  }
  delay(300);
  update_values();
  stop();
  turn_180();
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(1000);
  stop();
}

//going straight until the next single turn/junction (if already on junction, ignore it and go to the next one)
void go_forward()
{
  forward();
  update_values();
  while (fsr_val == HIGH || fsl_val == HIGH)
  {
    line_following();
    update_values();
  }
  delay(100);
  // set a previous state for security check to avoid an accidental detection of junction that might occur within short time domain
  bool previous_state = false;
  while (fsr_val == LOW && fsl_val == LOW || !previous_state) // no junction or single turn detected so go straight 
  {
    line_following();
    update_values();
    if (fsr_val == HIGH || fsl_val == HIGH){previous_state = true;delay(50); update_values();}
    else {previous_state = false;}
  }
  stop();
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}

void go_backward()
{
  backward();
  update_values();
  while (fsr_val == HIGH || fsl_val == HIGH)
  {
    backward_line_following();
    update_values();
  }
  delay(100);
  // set a previous state for security check to avoid an accidental detection of junction that might occur within short time domain
  bool previous_state = false;
  while (fsr_val == LOW && fsl_val == LOW || !previous_state) // no junction or single turn detected so go straight 
  {
    backward_line_following();
    update_values();
    if (fsr_val == HIGH || fsl_val == HIGH){previous_state = true;delay(50); update_values();}
    else {previous_state = false;}
  }
  stop();
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}


// servo motor control 
void angleforward(int x)
{
  for (int i = pos1; i <= pos1 + x; i += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(i);              // tell servo to go to position in variable 'i'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  pos1 += x; //update the global position value
}

void anglebackward(int x)
{
  for (int i = pos1; i >= pos1 - x; i -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(i);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  pos1 -= x; //update the global position value
}

// makes the box slide
void ramp_rotate() 
{ 
  angleforward(180);
  delay(3000);
  anglebackward(180);
}

// control the gate to allow box fall
void release() 
{ 
 for (pos2 = 0; pos2 <= 180; pos2 += 1) { // goes from 0 degrees to 180 degrees 
 // in steps of 1 degree 
 myservo2.write(pos2); // tell servo to go to position in variable 'pos' 
 delay(15); // waits 15 ms for the servo to reach the position 
 } 
 for (pos2 = 180; pos2 >= 0; pos2 -= 1) { // goes from 180 degrees to 0 degrees 
 myservo2.write(pos2); // tell servo to go to position in variable 'pos' 
 delay(15); // waits 15 ms for the servo to reach the position 
 } 
} 
// server motor control end

// push against the wall in order to pull up boxes
void destroy_the_wall()
{
  update_values();
  forward();
  while (crush_state == LOW)
  {
    line_following();
    update_values();
  }

  stop();
}

void route_to_factory() //hardcoded route to the factory (just gets there)
{
  
  update_values();
  go_forward();
  
  // 0 : forward; 1: left; 2:right
  // int route2factory[] = {0, 0, 1, 2, 0, 2, 2};
  int route2factory[] = {0};
  int arraylength = 1;
  // int arraylength = 9;
  // int arraylength = sizeof(route2factory) / sizeof(route2factory[0]);

  for (int i = 0 ; i < arraylength; i++){
    if (route2factory[i] == 0){
      go_forward();
    }
    else if (route2factory[i] == 1){
      swingTurnLeft();
      go_forward();
    }
    else if (route2factory[i] == 2){
      swingTurnRight();
      go_forward();
    }
  }
  swingTurnRight();
}

void factory_to_one()
{
  update_values();
  go_backward();
  
  // 0 : forward; 1: left; 2:right
  int route[] = {2, 1, 1, 1, 0};
  int arraylength = sizeof(route) / sizeof(route[0]);

  for (int i = 0 ; i < arraylength; i++){
    if (route[i] == 0){
      go_forward();
    }
    else if (route[i] == 1){
      swingTurnLeft();
      go_forward();
    }
    else if (route[i] == 2){
      swingTurnRight();
      go_forward();
    }
  }
}

void one_to_two()
{
  update_values();
  go_forward();
  // 0 : forward; 1: left; 2:right
  int route[] = {0, 0};
  int arraylength = sizeof(route) / sizeof(route[0]);

  for (int i = 0 ; i < arraylength; i++){
    if (route[i] == 0){
      go_forward();
    }
    else if (route[i] == 1){
      swingTurnLeft();
      go_forward();
    }
    else if (route[i] == 2){
      swingTurnRight();
      go_forward();
    }
  }
}

void two_to_three()
{
  update_values();
  go_forward();
  // 0 : forward; 1: left; 2:right
  int route[] = {2, 2, 0};
  int arraylength = sizeof(route) / sizeof(route[0]);

  for (int i = 0 ; i < arraylength; i++){
    if (route[i] == 0){
      go_forward();
    }
    else if (route[i] == 1){
      swingTurnLeft();
      go_forward();
    }
    else if (route[i] == 2){
      swingTurnRight();
      go_forward();
    }
  }
}

void three_to_four()
{
  update_values();
  go_forward();
  // 0 : forward; 1: left; 2:right
  int route[] = {0, 1};
  int arraylength = sizeof(route) / sizeof(route[0]);

  for (int i = 0 ; i < arraylength; i++){
    if (route[i] == 0){
      go_forward();
    }
    else if (route[i] == 1){
      swingTurnLeft();
      go_forward();
    }
    else if (route[i] == 2){
      swingTurnRight();
      go_forward();
    }
  }
}

void four_to_start()
{
  update_values();
  go_forward();
  destroy_the_wall();
}

// route from factory to area for magnetic box
void factory_to_red()
{
  update_values();
  go_forward();
  
  // 0 : forward; 1: left; 2:right
  int route2factory[] = {2, 1, 0, 0, 2, 0};
  // int arraylength = 6;
  int arraylength = sizeof(route2factory) / sizeof(route2factory[0]);

  for (int i = 0 ; i < arraylength; i++)
  {
    if (route2factory[i] == 0){
      go_forward();
    }
    else if (route2factory[i] == 1){
      swingTurnLeft();
      go_forward();
    }
    else if (route2factory[i] == 2){
      swingTurnRight();
      go_forward();
    }
  }
}

void red_to_one()
{
  update_values();
  go_forward();
  
  // 0 : forward; 1: left; 2:right
  int route2factory[] = {0, 1, 0, 0, 0};
  // int arraylength = 5;
  int arraylength = sizeof(route2factory) / sizeof(route2factory[0]);

  for (int i = 0 ; i < arraylength; i++)
  {
    if (route2factory[i] == 0){
      go_forward();
    }
    else if (route2factory[i] == 1){
      swingTurnLeft();
      go_forward();
    }
    else if (route2factory[i] == 2){
      swingTurnRight();
      go_forward();
    }
  }
}

void red_to_two()
{
  update_values();
  go_forward();
  
  // 0 : forward; 1: left; 2:right
  int route2factory[] = {0, 1};
  // int arraylength = 2;
  int arraylength = sizeof(route2factory) / sizeof(route2factory[0]);

  for (int i = 0 ; i < arraylength; i++)
  {
    if (route2factory[i] == 0){
      go_forward();
    }
    else if (route2factory[i] == 1){
      swingTurnLeft();
      go_forward();
    }
    else if (route2factory[i] == 2){
      swingTurnRight();
      go_forward();
    }
  }
}

void red_to_three()
{
  update_values();
  go_forward();
  
  // 0 : forward; 1: left; 2:right
  int route2factory[] = {1};
  // int arraylength = 6;
  int arraylength = sizeof(route2factory) / sizeof(route2factory[0]);

  for (int i = 0 ; i < arraylength; i++)
  {
    if (route2factory[i] == 0){
      go_forward();
    }
    else if (route2factory[i] == 1){
      swingTurnLeft();
      go_forward();
    }
    else if (route2factory[i] == 2){
      swingTurnRight();
      go_forward();
    }
  }
}

// testing: should be able to follow the line, turn at a single turn where there's no junction and finally stop at a junction
void loop()
{
  update_values();
  angleforward(180);
  route_to_factory();
  destroy_the_wall();
  ramp_rotate();
  factory_to_one();
  release();
  one_to_two;
  release();
  two_to_three();
  release();
  three_to_four();
  release();
  four_to_start();
}