/* Class MotorController:
   A class defining an interface to easily control the two motors of the segway
   via analog communication with the RobotEQ motor controller.
*/

#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

// Begin class MotorController
class MotorController {
public:
  MotorController() : DAC1_mult(1.0), DAC2_mult(1.0) { }

  /* MotorController::boot():
     Performs one-time setup functions for MotorController

     @params
     void

     @return
     void
  */
  void boot();
  
  /* MotorController::init():
     Ensures that the RobotEQ motor controller is initialized to Analog Control
     Mode with a constant 2.5V signal for 1 sec. 

     @params
     void

     @return
     void
  */
  void init();

  /* MotorController::write():
     Sends motor control output from PID calculation to the RoboteQ
     motor controller
  
     @params
     spd:       The speed for the motors (in range (-255, 255), signed)
     indicating direction of rotation.
     -255 - -1    = Backward
     0            = Stop
     1 - 255      = Forward

     @return
     void
  */
  void write(int spd);

  double getLeftSpeed(){ return DAC1_mult; }
  double getRightSpeed(){ return DAC2_mult; }
  
  double setLeftSpeed(double spd){
    return DAC1_mult =   (spd < 0.0 || spd > 1.0)  ?  DAC1_mult  :  spd;
  }
  double setRightSpeed(double spd){
    return DAC2_mult =   (spd < 0.0 || spd > 1.0)  ?  DAC2_mult  :  spd;
  }
  
private:
  double DAC1_mult, DAC2_mult;

  /* DAC():
     Controls one Digital to Analog Converter to ouput the given voltage analog
     signal using SPI to communicate with the DAC.

     @params
     dac:            The DAC chip to control
     volt:           The voltage analog signal to be output

     @return
     void
  */
  void DAC(int dac, double volt);
};
// End class MotorController

#endif
