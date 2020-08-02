#include "encoders.h"
#include "pid.h"
#include "kinematics.h"
#include "lineSensors.h"

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define LINE_LEFT_PIN   A2 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN  A4 //Pin for the right line sensor
#define BUZZER 6
#define kpReturn 3.50
#define kiReturn 0.00025
#define kdReturn 0.05
#define kpLineFF 5.25
#define kiLineFF 0.05
#define kdLineFF 0.15
 
unsigned long startTime;
unsigned long prevUpdate;
unsigned long prevMove;
unsigned long timeStamp;
long prevleftCount;
long prevrightCount;
float homeTheta;
int leftSensorRead,  centreSensorRead,  rightSensorRead; //define sensor readings
int STATE;
bool setupDist;
bool setupCheck;

PID pidReturn( kpReturn, kiReturn, kdReturn );
PID pidLineFF( kpLineFF, kiLineFF, kdLineFF );

lineSensor line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
lineSensor line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
lineSensor line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

Kinematics kinematics;


void setup() 
{
  // Initialise your other globals variables
  // and devices.

  setupEncoder0();
  setupEncoder1();

  line_left.calib();
  line_centre.calib();
  line_right.calib();

  STATE = 0;
  setupDist = false;
  setupCheck = false;

  startTime = millis();
  prevUpdate = millis(); 
  prevMove = millis();
  timeStamp = micros();
  prevleftCount = 0;
  prevrightCount = 0;

  leftSensorRead = 0;
  centreSensorRead = 0;
  rightSensorRead = 0;
  
  // Initialise the Serial communication
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
}


// Remember, loop runs again and again
void loop(){
  unsigned long currentTime = millis();
  unsigned long update_time = currentTime - prevUpdate;
  unsigned long moveTime = currentTime - prevMove;

  if (update_time > 2) {
    prevUpdate = currentTime;
    kinematics.update(countLeft, countRight); // call an update to your kinematics at a time interval
  }


  if (moveTime > 5) {
    prevMove = currentTime;
    Serial.println(STATE);
    switch (STATE) {
      case 0: 
        FindLine();
        break;
      case 1:
        BangBang();
        break;
      case 2:
        RejoinLine();
        break;
      case 3:
        FaceHome();
        break;
      case 4:
        DriveHomeX();
        break;
      case 5:
        turnLEFT2home(90.0f);
        break;
      case 6:
        DriveHomeY();
        break;
      case 7:
        leftMotor(0.0f);
        rightMotor(0.0f);
        Serial.println("Obstacle course finished");
      default:
        Serial.println("Unsteady, Code Red!");
        break;
    }
  }

}

//Move forwards towards line.
  void FindLine() {

  bool onLine = checkForLine();

  if (!onLine) {
    float theta_error = kinematics.getTheta();
    int turn_pwm = 0;
  
    if (theta_error < 0){
      turn_pwm = -2;
    }
    else if (theta_error > 0) {
      turn_pwm = 2;
    }
    else turn_pwm = 0;
  
    int left_demand = 50 - turn_pwm;
    int right_demand = 50 + turn_pwm;
  
    leftMotor(left_demand);
    rightMotor(right_demand);
  }
  else {
    leftMotor(0.0f);
    rightMotor(0.0f);

    //Set STATE to follow line.
    STATE = 1;
  }
}

//Initial check for line.
bool checkForLine() {

  bool onLine = false;

  leftSensorRead = line_left.readCalib();
  centreSensorRead = line_centre.readCalib();
  rightSensorRead = line_right.readCalib();

  if ( leftSensorRead > 300 || centreSensorRead > 300 || rightSensorRead > 300 ) {
    onLine = true;
  }

  return onLine;   
}


// Bang Bang Line Follower
void BangBang() {
  
  leftSensorRead = line_left.readCalib();
  centreSensorRead = line_centre.readCalib();
  rightSensorRead = line_right.readCalib();

  bool left_on_line = false;
  bool centre_on_line = false;
  bool right_on_line = false;
  
  if (leftSensorRead > 90) left_on_line = true;
  if (centreSensorRead > 110) centre_on_line = true;
  if (rightSensorRead > 90) right_on_line = true;

  if (centre_on_line) {
    leftMotor(37.0f);
    rightMotor(36.0f);
  }
  else if (left_on_line) {
    rightMotor(36.0f);
    leftMotor(-37.0f);
  }
  else if (right_on_line) {
    leftMotor(37.0f);
    rightMotor(-36.0f);
  }
  else {
    leftMotor(0.0f);
    rightMotor(0.0f);

    //Try to rejoin line
    STATE = 2;
  }

}

//Try to find line if lost
  bool RejoinLine() {

  bool FoundLine = false;

  unsigned long currentTime = millis();
  
  if (!setupCheck) {
    startTime = millis();
    setupCheck = true;
  }

  unsigned long elapsedTime = currentTime - startTime;

  if (elapsedTime < 1000) {
    leftMotor(-32.0f);
    rightMotor(31.0f);
    FoundLine = checkForLine();
  }
  else if (elapsedTime < 3000) {
    leftMotor(32.0f);
    rightMotor(-31.0f);
    FoundLine = checkForLine();
  }
  else {
    leftMotor(0.0f);
    rightMotor(0.0f);
    analogWrite(BUZZER, 10);
    delay(1000);
    analogWrite(BUZZER, 0);
    STATE = 3;
  }
  
  if (FoundLine) {
    setupCheck = false;
    STATE = 1;
  }
}


//Turn and face home.
void FaceHome() {
  float angle = kinematics.homeAngle();
  turn_angle_right(angle);
}

void DriveHomeX() {
  
  if (!setupDist) {
    homeTheta = kinematics.getTheta();
    setupDist = true;
  }

  float theta_error = homeTheta - kinematics.getTheta();
  int turn_pwm = 0;

  if (theta_error > 0){
    turn_pwm = -2;
  }
  else if (theta_error < 0) {
    turn_pwm = 2;
  }
  else turn_pwm = 0;

  int left_demand = 50 - turn_pwm;
  int right_demand = 50 + turn_pwm;

  leftMotor(left_demand);
  rightMotor(right_demand);

  if (abs(kinematics.getXpos()) < 10) {
    setupDist = false;
    STATE = 5;
  }
  
}

//After turning 90 degrees, drive until y position = 0
void DriveHomeY() {
  
  if (!setupDist) {
    homeTheta = kinematics.getTheta();
    setupDist = true;
  }

  float theta_error = homeTheta - kinematics.getTheta();
  int turn_pwm = 0;

  if (theta_error > 0){
    turn_pwm = -2;
  }
  else if (theta_error < 0) {
    turn_pwm = 2;
  }
  else turn_pwm = 0;

  int left_demand = 100 - turn_pwm;
  int right_demand = 100 + turn_pwm;

  leftMotor(left_demand);
  rightMotor(right_demand);

  if (abs(kinematics.getYpos()) < 60) {
    STATE = 7;
  }
 Serial.println( homeTheta );
 }

//Turn to face start of map
void turn_angle_right(float angle) {

  float new_countLeft, new_countRight;

  if (!setupDist) {
    float count = (((angle / 360.0f) * (140.0f * PI)) / (70.0f * PI)) * 1440.0f;
    new_countLeft = countLeft + count;
    new_countRight = countRight - count;
    setupDist = true;
  }

  float speed = 75.0f;
  
  if (countLeft < new_countLeft) {
    leftMotor(speed + 1.0f);
  }
  else leftMotor(0.0f);

  if (countRight > new_countRight) {
    rightMotor(-speed);
  }
  else rightMotor(0.0f);

  if (countLeft > new_countLeft && countRight < new_countRight) {
    setupDist = false;
    
    //Set STATE to drive towards home.
    STATE = 4;
  }
}

//Turn to face home
void turnLEFT2home(float angle) {

  float new_countLeft, new_countRight;

  if (!setupDist) {
    float count = (((angle / 360.0f) * (140.0f * PI)) / (70.0f * PI)) * 1440.0f;
    new_countLeft = countLeft - count;
    new_countRight = countRight + count;
    setupDist = true;
  }

  Serial.print(countLeft);
  Serial.print( ", " );
  Serial.println(countRight);
  Serial.print(new_countLeft);
  Serial.print( ", " );
  Serial.println(new_countRight);

  float speed = 75.0f;
  
  if (countLeft > new_countLeft) {
    leftMotor(-(speed + 1.0f));
  }
  else leftMotor(0.0f);

  if (countRight < new_countRight) {
    rightMotor(speed);
  }
  else rightMotor(0.0f);

  if (countLeft <= new_countLeft && countRight >= new_countRight) {
    setupDist = false;
    STATE = 6;
  }
}


// Defining motor functions
void leftMotor(float speed) {
  if (speed < -255.0f || speed > 255.0f) {
    Serial.println("Invalid Left Motor Speed.");
  }
  else {
    if (speed >= 0) digitalWrite( L_DIR_PIN, LOW );
    else digitalWrite( L_DIR_PIN, HIGH );

    speed = abs(speed);
    analogWrite( L_PWM_PIN, speed );
  }
}

void rightMotor(float speed) {
  if (speed < -255.0f || speed > 255.0f) {
    Serial.println("Motor speed is no.");
  }
  else {
    if (speed >= 0) digitalWrite( R_DIR_PIN, LOW );
    else digitalWrite( R_DIR_PIN, HIGH );

    speed = abs(speed);
    analogWrite( R_PWM_PIN, speed );
  }
}
