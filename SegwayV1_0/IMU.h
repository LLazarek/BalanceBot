#ifndef IMU_H
#define IMU_H

#include "Gyro.h"
#include "Accel.h"

class IMU {
public:
  IMU(int Loop_Time) : angle(0.0), bias_sample_size(20), loop_time(Loop_Time) {}
  
  /* IMU::init():
     Performs bias initialization for gyro and accel.
     (See: Gyro::calcBias(), Accel::calcBias())

     @params
     void

     @return
     void
  */
  void init();

  /* IMU::readFilteredAngle():
     Reads data from the gyro and accel, then applies a complementary filter to fuse the two readings into a current-angle estimate.

     @params
     void
  
     @return
     double               The estimated current angle of the robot
  */
  double readFilteredAngle();

private:
  Gyro gyro;
  Accel accel;
  double angle;
  const int bias_sample_size;
  const int loop_time; // in ms
};

#endif