#include "Accel.h"
#include "serialpp.h"

void Accel::calcBias(int sampleNum){
  bias = 0;
  
  for(int n = 0; n < sampleNum; n++)
  {
    this -> read();
    bias += this -> a.z >> 4;// last 4 bits are 0
    delay(50);
  }
  bias = bias/sampleNum;
  Print1("\tAccel Bias: ", bias, "\n");
}

double Accel::readAngle(){
  double accel_y;

  this -> read();
  accel_y = this -> a.z >> 4;
  return (accel_y - bias)*sensitivity;
}
