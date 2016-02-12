#ifndef HEAD_H
#define HEAD_H

#include "IMU.h"
#include "myPID.h"
#include "MotorController.h"

#define loop_time 10 // in ms
#define LEDpin 2
#define switchPin 7

#define PRINT_STATUS  Print5("K vals: kp = ", pid.getKp(), ", ki = ",\
			     pid.getKi(),", kd = ", pid.getKd(),\
			     "\nleft = ", mc.getLeftSpeed(), ", right = ",\
			     mc.getRightSpeed(), "\n")

int fallen = 0;// Flag for fall detection

IMU imu(loop_time);
myPID pid(2, 0, 0); // p, i, d
MotorController mc;

bool kill = 0;

/************************** Begin Functions ************************/

/* handleInput():
   Reads data from the Serial moniter and interprets commands to update
   PID tuning variables during runtime

   Valid input format: "kp 55.6", "kp53",  "p 53", "p55.6", "ki 53"...
   Invalid input is thrown away

   Additional input accepted:
   "l1.2" = update multiplier for DAC1 to be 1.2
   "r0.8" = update multiplier for DAC2 to be 0.8
   "s"    = emergency override/kill the segway until switch is flipped OFF

   @params
   void

   @return
   void
*/
void handleInput(){
  char var;
  double res;
  
  var = serialpp::readChar();
  if(var != -1 && var == 's'){
    Print("Emergency software kill/override activated.\n");
    kill = true;
  }else if(var != -1 &&
	   (var == 'p' || var == 'i' ||var == 'd' ||var == 'l' || var == 'r')){
    res = serialpp::readDouble();
    serialpp::rmWhiteSpc();

    if(var == 'p'){
      Print2("Changed kp value from ", pid.getKp(), " to ", res, "\n");
      pid.setTunings(res, pid.getKi(), pid.getKd());
    }else if(var == 'i'){
      Print2("Changed ki value from ", pid.getKi(), " to ", res, "\n");
      pid.setTunings(pid.getKp(), res, pid.getKd());
    }else if(var == 'd'){
      Print2("Changed kd value from ", pid.getKd(), " to ", res, "\n");
      pid.setTunings(pid.getKp(), pid.getKi(), res);
    }else if(var == 'l'){
      Print1("Changed left wheel mult from ", mc.getLeftSpeed(), " to ");
      Print1("", mc.setLeftSpeed(res), "\n");
    }else if(var == 'r'){
      Print1("Changed right wheel mult from ", mc.getRightSpeed(), " to ");
      Print1("", mc.setRightSpeed(res), "\n");
    }
    PRINT_STATUS;
  }else if(var == '.'){
    PRINT_STATUS;
  }
}

/* reset():
   Re-initializes all necessary variables etc to restart the
   balancing algorithm

   @params
   void

   @return
   void
*/
void reset(){
  fallen = 0;
  imu.init();
  pid.reset();
  mc.init();
}

#endif
