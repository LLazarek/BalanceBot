#include "Accel.h"
#include "serialpp.h"

void Accel::boot(){
  Wire.begin();
  pinMode(A0, INPUT); // Accel pin = A0
  while(!(this -> init())){
    Serial.println("Can't load accel");
    delay(1000);
  }
  this -> enableDefault();
}

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
