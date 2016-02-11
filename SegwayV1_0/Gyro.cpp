#include "Gyro.h"
#include "serialpp.h"

void Gyro::boot(){
  while(!(this -> init())){
    Print("Unable to load gyro, please check IMU connection");
    delay(1000);
  }
  this -> enableDefault();
}

void Gyro::calcBias(int sampleNum){
  bias = 0;
  
  for(int n = 0; n < sampleNum; n++)
  {
    this -> read();
    bias += this -> g.y;
    delay(50);
  }
  bias = bias/sampleNum;
  Print1("Gyro Bias: ", bias, "");
}

double Gyro::readRate(){
  double gyro_velocity;
  
  this -> read();
  gyro_velocity = (this -> g.y - bias) * DPS_conversion_factor;
  return gyro_velocity;
}
