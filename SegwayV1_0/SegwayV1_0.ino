/* Changelog: SegwayV1.0

* Motor control transferred to RoboteQ controller
  motorControl() now converts & sends speed to RoboteQ
* Moved integration of gyro angular ROC from 
  gyro_readDAngle() to filter()
* Renamed gyro_readDAngle() to gyro_readRate() to
  reflect the change
* Replaced reset function to simulate a program restart
  rather than actually restarting the program
  (see: reset() in Head.h)

** Summary ** (- = removed, + = added, ~ = changed)
  - softwareReset()    - gyro_readDAngle()
  + reset()            + gyro_readRate()

  ~ motorControl()     ~ filter()
*/

#include <Wire.h>
#include <L3G.h>
#include <PID_v1.h>
#include <LSM303.h>
#include <SPI.h>
#include "Head.h"// Contains function definitions

/********************** SETUP **********************/
void setup(){
  pinMode(accel_y_pin, INPUT);
  pinMode(switchPin, INPUT_PULLUP);// Engage pullup resistor
  pinMode(LEDpin, OUTPUT);
  pinMode(DAC1, OUTPUT);
  pinMode(DAC2, OUTPUT);
  digitalWrite(DAC1, HIGH);// default DAC ss pin state
  digitalWrite(DAC2, HIGH);
  
  Serial.begin(115200);
  Wire.begin();
  SPI.begin();
  SPI.beginTransaction(SPISettings(14000000, 
  MSBFIRST, SPI_MODE0));
  
  Print3("K vals: kp = ", kp, ", ki = ", ki, ", kd = ", kd, "\n");

  while(digitalRead(switchPin)) delay(250);

  // Initialize Gyro
  while(!gyro.init());
  gyro.enableDefault();

  while(!compass.init());
  compass.enableDefault();
  
  /* Stabilization delay */
  Println("Stabilize the robot.");
  motorControl(0);
  delay(2000);

  biasInit();
  Print("A");
  PID_init(); 
  Print("B");
  robotEQ_init();
  
  digitalWrite(LEDpin, HIGH);// Ready to go
  
  Println("FiltAngle\tPIDOut\tFallen");// Labels for data analysis
}


/*********************** LOOP ************************/
void loop(){
  static int prevSwState = 0;// Power switch state tracker
  static double gyro_rate, accel_angle;
  
  /* Switch handling: Stops running if the switch is flipped off,
     then resets sketch when flipped back on - see reset() */
  while(digitalRead(switchPin)){// Loop while switch is off
    if(prevSwState == 0){
      digitalWrite(LEDpin, LOW);
      prevSwState = 1;
      motorControl(0);
      Println("Robot is now off. Flip the switch to turn on.");
      Print3("kp = ", kp, ", ki = ", ki, ", kd = ", kd, "\n");
    }
    
    if(checkSerialMon()) updateTunings();// Handle serial input
    
    delay(250);
  }
  if(prevSwState == 1){// Switch turned back on
    reset();// See Head.h
    digitalWrite(LEDpin, HIGH);
    prevSwState = 0;
  }

  
  if(checkSerialMon()) updateTunings();
  
  gyro_rate = gyro_readRate();
  accel_angle = accel_readAngle();
  
  input = filter(gyro_rate, accel_angle);// Filter angle reading
  Print(input);
  myPID.Compute();
  output = PID_Hist(output);// Smooth PID output
  Print1("\t", output, "");
  
  if(input < -17 || input > 30){// Robot has fallen over
    fallen = 1;
    motorControl(0);// Cut the motors
  }
  
  if(!fallen) motorControl(output);
  else Print("\tFallen");
  
  Println();
  delay(loop_time);
}
