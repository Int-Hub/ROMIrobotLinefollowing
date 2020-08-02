#ifndef _Line_follow_h
#define _Line_follow_h
#define BUZZER 6

//Number of readings to take for calibration
const int numCalib = 100;

class lineSensor {
  public:

    // Required function.
    lineSensor(int pin);   //Constructor

    // Suggested functions.
    void calib();       //Calibrate
    int readRaw();         //Return the uncalibrated value from the sensor
    int readCalib();  //Return the calibrated value from the sensor

  private:
    int pin;
    int avg;
};

// Class Constructor: 
// Sets pin passed in as argument to input
lineSensor::lineSensor(int linePin) {
  pin = linePin;
  pinMode(pin, INPUT);
}

// Returns unmodified reading.
  int lineSensor::readRaw() {
  return analogRead(pin);
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void lineSensor::calib() {
    long add = 0;
    for (float  j  = 0; j < numCalib; j = j+ 1) {
    add = add + analogRead(pin);
  }

  avg = add / numCalib;
  analogWrite(BUZZER, 10);
  delay(100);
  analogWrite(BUZZER, 0);
  
}

// calibrated sensor reading.

   int lineSensor::readCalib() {  
   int reading = analogRead(pin) - avg;
   reading = constrain(reading, 0, 1023);
   return reading;
}


#endif
