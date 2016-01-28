/* Class Gyro:
   Derived class of L3G gyro adding specialized bias and read functions.
*/

#ifndef GYRO_H
#define GYRO_H

#include <L3G.h>

// Begin class Gyro
class Gyro : public L3G {
public:
  Gyro() : bias(0.00) {
    while(!(this -> init())){
      Serial.println("Can't load gyro");
      delay(1000);
    }
    this -> enableDefault();
  }
  
  /* Gyro::calcBias():
     Calculates the gyroscope bias
     (See: sampleNum)

     @params
     int sampleNum        The number of samples to use in bias calculation

     @return
     void
  */
  void calcBias(int sampleNum);

  /* Gyro::readRate():
     Reads gyro angular velocity data
     (See: gyro_bias, gyro_calcBias(), gyro_conversion_DPS)

     @params
     void

     @return
     The approximate angular rate of change
  */
  double readRate();
  
private:
  double bias;
  const double DPS_conversion_factor = 8.75/1000; // Conversion factor to angular velocity
};
// End class Gyro

#endif
