/* Class Accel:
   Derived class of LSM303 adding specialized bias and read functions.
*/

#ifndef ACCEL_H
#define ACCEL_H

#include <Wire.h>
#include <LSM303.h>

// Begin class Accel
class Accel : public LSM303 {
public:
  Accel(): bias(0.00) {
    Wire.begin();
    pinMode(A0, INPUT); // Accel pin = A0
    while(!(this -> init())){
      Serial.println("Can't load accel");
      delay(1000);
    }
    this -> enableDefault();
  }
  
  /* Accel::calcBias:
     Calculates the accelerometer bias
     (See: sampleNum)

     @params
     int sampleNum        The number of samples to use in bias calculation

     @return
     void
  */
  void calcBias(int sampleNum);

  /* Accel::readAngle:
     Reads accelerometer data and converts it into an approximate angle
     (See: accel_bias, accel_calcBias(), accel_sensitivity)

     @params
     void

     @return
     The approximate angle reading
  */
  double readAngle();
  
private:
  double bias;
  const double sensitivity = 1/17.28; // LSM303 sensitivity
};
// End class Accel

#endif
