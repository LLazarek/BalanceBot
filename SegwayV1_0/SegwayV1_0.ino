/* Changelog: SegwayV1.0

*/

#include "serialpp.h"
#include "Head.h"

/********************** SETUP **********************/
void setup(){
  Serial.begin(115200);
  pinMode(switchPin, INPUT_PULLUP);// Engage pullup resistor
  pinMode(LEDpin, OUTPUT);

  /* boot() functions must be invoked within setup() (as opposed to in the
     objects' respective constructors) to avoid conflict with the arduino's own
     initialization process - globals are constructed before the arduino is
     fully ready to execute some types of commands.
   */
  imu.boot();
  mc.boot();
  
  PRINT_STATUS;

  while(digitalRead(switchPin)) delay(100);

  /* Stabilization delay */
  Println("Stabilize the robot.");
  mc.init();
  mc.write(0);
  delay(2000);

  imu.init();
  pid.init();
  
  digitalWrite(LEDpin, HIGH);// Ready to go
  
  Println("FiltAngle\tPIDOut\tFallen");// Labels for data analysis
}


/*********************** LOOP ************************/
void loop(){
  static int prevSwState = 0;// Power switch state tracker
  static int prevKillState = 0;// Kill/override state tracker
  static double gyro_rate, accel_angle;

  while(kill){
    if(kill && prevKillState == 0){
      digitalWrite(LEDpin, LOW);
      prevKillState = 1;
      mc.write(0);
      Println("Emergency software kill has been activated; robot is now off."
	      " Flip the switch to OFF to re-enter normal execution.");
      PRINT_STATUS;
    }

    // Exit emergency kill loop once switch has been flipped to OFF
    if(digitalRead(switchPin)) kill = prevKillState = 0;
    
    if(serialpp::inputAvailable()) handleInput();
    
    delay(250);
  }
  
  /* Switch handling: Stops running if the switch is flipped off,
     then resets sketch when flipped back on - see Head.h/reset() */
  while(digitalRead(switchPin)){// Loop while switch is off
    if(prevSwState == 0){
      digitalWrite(LEDpin, LOW);
      prevSwState = 1;
      mc.write(0);
      Println("Robot is now off. Flip the switch to turn on.");
      PRINT_STATUS;
    }
    
    if(serialpp::inputAvailable()) handleInput();
    
    delay(250);
  }
  if(prevSwState == 1){// Switch turned back on from being off
    reset();// See Head.h
    digitalWrite(LEDpin, HIGH);
    prevSwState = 0;
  }

  
  if(serialpp::inputAvailable()) handleInput();
    
  /*  pid.setInput(imu.readFilteredAngle());// Filter angle reading
  Print(pid.getInput());
  pid.Compute();
  pid.setOutput(pid.updateHist(pid.getOutput()));// Smooth PID output
  Print1("\t", pid.getOutput(), "");*/

  double angle = imu.readFilteredAngle();
  Print(angle);
  if(angle < -17 || angle > 30){ // Robot has fallen over
    fallen = 1;
    mc.write(0); // Cut the motors
  }
  
  if(!fallen){
    mc.write(pid.compute(angle));
    Print1("\t", pid.getOutput(), "");
  }
  else Print("\tFallen");

  Print1("\tPot: ", analogRead(A0), "");
  
  Println();
  delay(loop_time);
}
