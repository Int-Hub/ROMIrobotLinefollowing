#ifndef _PID_h
#define _PID_h
#include <stdint.h>


class PID {
  
  public:

    PID(float P, float I, float D);                 // This is the class constructor. It is called whenever we create an instance of the PID class 
    void setGains(float P, float I, float D );      // This function updates the values of the gains
    void reset();                                   // This function resets any stored values used by the integral or derative terms
    float update(float demand, float measurement);  // This function calculates the PID control signal. It should be called in a loop
    void printComponents();                        // This function prints the individual components of the control signal and can be used for debugging
    void setMax(float newMax);                     // This function sets the maximum output the controller can ask for
    void setDebug(bool state);                      // This function sets the debug flag;
    void printResponse();                          // This function prints the ratio of input to output in a way that is nicely interpreted by the Serial plotter
    void setShowResponse(bool state);             // This functions set the show_response flag

  private:

    //Control gains
    float Kp; //Proportional gain
    float Ki; //Integral gain
    float Kd; //Derivative gain

    //We can use this to limit the output to a certain value
    float maxOutput = 255; 

    //Output components
    //These are used for debugging purposes
    float Kp_output = 0; 
    float Ki_output = 0;
    float Kd_output = 0;
    float totalOutput = 0;

    //Values to store between updates().
    float lastDemand = 0;            //For storing the previous input
    float lastMeasurement = 0;       //For storing the last measurement
    float lastError = 0;             //For calculating the derivative term
    float integralError = 0;         //For storing the integral of the error
    long  lastMillis = 0;            //To track elapsed_time
    bool  debug = false;             //This flag controls whether we print the contributions of each component when update is called
    bool  showResponse = false;     // This flag controls whether we print the response of the controller on each update
    float timeDelta;
};

 PID::PID(float P, float I, float D)
{
  //Store pid gains
  setGains(P,I,D);
  long lastMillis = millis();
}
//
//void PID::printComponents(){
//  Serial.print(" proportionalTerm: ");
//  //Serial.print(kp_output);
//  Serial.print(" integralTerm: ");
////Serial.print(kd_output);
//  Serial.print(" derivativeTerm: ");
//  //Serial.print(ki_output);
//  Serial.print(" totalOutput: ");
//  //Serial.println(totalOutput);
//}

/*
 * This function sets the gains of the PID controller
 */
void PID::setGains(float P, float I, float D) {
  Kp = P;
  Ki = I;
  Kd = D;
}

float PID::update(float demand, float measurement) {
  //Calculate how much time (in milliseconds) has passed since the last update call
  long timeNow = millis();
  int timeDelta = timeNow - lastMillis;
  lastMillis = timeNow;
  
  //This represents the error term
  float error;
  error = demand - measurement;   
  
  //This represents the error derivative
 float errorDelta;
 errorDelta= (error - lastError)/float(timeDelta);
 lastError = error;

  // This represents the error integral.
  // Integrate error over time.
  integralError = integralError + (error * float(timeDelta));

  //Attenuate above error components by gain values.
  Kp_output = Kp * error;
  Ki_output = Ki * integralError;
  Kd_output = Kd * errorDelta;

  // Add the three components to get the total output

  float totalOutput = Kp_output + Ki_output + Kd_output;

  /*
   * ===========================
   * Code below this point should not need to be changed
   * But of course, feel free to improve / experiment :)
   */

   
  //Update persistent variables.
  lastDemand = demand;
  lastMeasurement = measurement;

  // Catching max in positive sign.
  if (totalOutput > maxOutput) {
    totalOutput = maxOutput;
  } 

  // Catching max in negative sign
  if (totalOutput < -maxOutput) {
    totalOutput = -maxOutput;
  }

  //Print debugging information if required
  if (debug) {
    Serial.print("error:");
    Serial.print(error);
    Serial.print("errorDelta:");
    Serial.print(errorDelta);
    Serial.print("integralError:");
    Serial.print(integralError);
    printComponents();
  }

  //Print response if required
  if (showResponse) {
    printResponse();
  }
  
  return totalOutput;
}

void PID::setMax(float newMax)
{
  if (newMax > 0) {
    maxOutput = newMax;
  } else {
    Serial.println("Maximum output must be strictly +ve");
  }
}

void PID::setDebug(bool state) {
  debug = state;
}

void PID::reset() {
  
  lastError = 0;
  integralError = 0;
  lastMillis = millis();
  
}

//This function prints measurement / demand - Good for visualiser the response on the Serial plotter
void PID::printResponse() {
  float response = lastMeasurement / lastDemand;
  Serial.println(response);
}

void PID::setShowResponse(bool state) {
  showResponse = state;
}
#endif
