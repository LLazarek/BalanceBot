#include "myPID.h"
#include "serialpp.h"

void myPID::init(){
  this -> SetMode(AUTOMATIC);
  this -> SetSampleTime(10); // 10 ms loop
  this -> SetOutputLimits(-255, 255);
}

double myPID::updateHist(double output){
  static int h = 0;
  double sum = 0;

  hist[h] = output;
  h = (h + 1)%histLen;

  for(int i = 0; i < histLen; ++i){
    sum += hist[i];
  }
  return sum/histLen;
}

void myPID::reset(){
  for(int i = 0; i < histLen; i++) hist[i] = 0;
  this -> SetMode(MANUAL);
  // Force PID I-term and output to 0
  this -> SetOutputLimits(0.0, 1.0);// Force min to 0
  this -> SetOutputLimits(-1.0, 0.0);// Force max to 0
  init(); // Re-initialize
}
