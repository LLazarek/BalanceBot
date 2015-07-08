/* Changelog: FilterV4_Due

* Updated Code to be Due compatible
* Changed motor control to RoboteQ controller

*/

#include <Wire.h>
#include <L3G.h>
#include <PID_v1.h>
#include <Adafruit_MotorShield.h>
#include "Head.h"

double gyro_rate, accel_angle;

/********************** SETUP **********************/
void setup(){
  pinMode(accel_y_pin, INPUT);
  pinMode(switchPin, INPUT_PULLUP);// Engage pullup resistor
  pinMode(LEDpin, OUTPUT);
  
  Serial.begin(115200);
  Wire.begin();
  
  print3("K vals: kp = ", kp, ", ki = ", ki, ", kd = ", kd, "\n");
  delay(2000);// Allow time for user to flip switch to edit tunings
  
  /* while(digitalRead(switchPin)){// Wait for switch to be flipped on */
  /*   if(checkSerialMon()) updateTunings();// Handle serial input */
    
  /*   delay(250); */
  /* } */
  
  // Initialize Gyro
  while(!gyro.init());
  gyro.enableDefault();
  
  /* Stabilization delay */
  println("Stabilize the robot.");
  motorControl(0);
  delay(2000);
  
  biasInit();
  PID_init(); 
  AFMS.begin();
  
  digitalWrite(LEDpin, HIGH);// Ready to go
  
  println("FiltAngle\tPIDOut\tFallen");// Labels for data analysis
}


/*********************** LOOP ************************/
void loop(){
  static int prevSwState = 0;// Power switch state tracker
  //static double gyro_d_angle, accel_angle;
  static double res = 0;
  
  /* Switch handling: Stops running if the switch is flipped off,
     then restarts sketch when flipped back on */
  /* while(digitalRead(switchPin)){// Loop while switch is off */
  /*   if(prevSwState == 0){ */
  /*     digitalWrite(LEDpin, LOW); */
  /*     prevSwState = 1; */
  /*     motorControl(0); */
  /*     println("Robot is now off. Flip the switch to turn on."); */
  /*     print3("kp = ", kp, ", ki = ", ki, ", kd = ", kd, "\n"); */
  /*   } */
  /*   delay(500); */
  /* } */
  /* if(prevSwState == 1){// Switch turned back on */
  /*   software_Reset();// See std.h */
  /* } */
  
  
  if(checkSerialMon()) updateTunings();
  gyro_rate = gyro_readRate();
  accel_angle = accel_readAngle();
  
  input = filter(gyro_rate, accel_angle);// Filter angle reading
  print(input);
  myPID.Compute();
  output = PID_Hist(output);// Smooth PID output
  print1("\t", output, "");
  
  if(input < -17 || input > 30){// Robot has fallen over
    fallen = 1;
    motorControl(0);// Cut the motors
  }
  
  if(!fallen) motorControl(output);
  else print1("\t", fallen, "");
  
  println();
  delay(loop_time);
}
