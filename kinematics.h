#ifndef _Kinematics
#define _Kinematics_h

#define PI 3.142

//You may want to use some/all of these variables
const float wheelDia = 69.5f; //mm
const float wheelRad = 34.75f; //mm
const float wheelSep = 140.0f;
const float gearRatio = 0.0083f;
const float COUNTS_PER_SHAFT_REVOLUTION = 12.0f;
const float COUNTS_PER_WHEEL_REVOLUTION =  1440.0f;
const float COUNTS_PER_MM  = (1 / (wheelDia * PI)) * COUNTS_PER_WHEEL_REVOLUTION;


// Build up your Kinematics class.
class Kinematics
{
  public:
    
    Kinematics();   // Constructor, required.

    void update(long countLeft, long countRight);  // should calucate an update to position.
    float homeAngle();
    float homeDistance();
    void  resetTheta();
    float getTheta();
    float getXpos();
    float getYpos();
    
  private:
    
    //Private variables and methods go here
    float xpos;
    float ypos;
    float theta;

    long old_countLeft;
    long old_countRight;
    
};


// Required constructor.  Initialise variables.
Kinematics::Kinematics() {
  xpos =  0;
  ypos =  0;
  theta = 0;

  old_countLeft = 0;
  old_countRight = 0;

}

void Kinematics :: update(long countLeft, long countRight) {

  long change_left = countLeft - old_countLeft;
  long change_right = countRight - old_countRight;

  float leftDist = (float)change_left / COUNTS_PER_MM;
  float RightDist = (float)change_right / COUNTS_PER_MM;

  float avgDist = (leftDist + RightDist) / 2.0f;
  
  xpos = xpos + avgDist * cos(theta);
  ypos = ypos + avgDist * sin(theta); 

  theta = theta + (leftDist - RightDist) / wheelSep;

  old_countLeft = countLeft;
  old_countRight = countRight;
  return;
}


float Kinematics :: homeAngle() {
  float angle = (- theta / (2 * PI) * 360) + 180;
  //float angle = atan2(ypos, xpos);
  return angle;
}


  float Kinematics :: homeDistance() {
  float distance = sqrt(sq(xpos) + sq(ypos));
  return distance;
}


void Kinematics :: resetTheta() {
  theta = 0;
}


float Kinematics :: getTheta() {
  return theta;
}

float Kinematics :: getXpos() {
  return xpos;
}

float Kinematics :: getYpos() {
  return ypos;
}

#endif
