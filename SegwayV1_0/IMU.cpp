#include "IMU.h"
#include "serialpp.h"

void IMU::init(){
  angle = 0.0;
  Println("Beginning bias initialization...");
  gyro.calcBias(bias_sample_size);
  accel.calcBias(bias_sample_size);
  Println("Initialization complete.");
}

double IMU::readFilteredAngle(){
  double gyro_rate = gyro.readRate(), accel_angle = accel.readAngle();
  angle = (gyro_rate*loop_time/1000 + angle)*0.97 + accel_angle*0.03;
  return angle;
}
