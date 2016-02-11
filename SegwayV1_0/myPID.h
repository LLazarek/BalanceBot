/* Class myPID:
   A class defining an interface to easily control a PID implementation
   with settings tailored to the application (segway).
*/

#ifndef MYPID_H
#define MYPID_H

#include <PID_v1.h>

// Begin class myPID
class myPID : public PID {
public:
  /* myPID::Constructor:
     Constructs the PID object with the given tuning parameters.

     @params
     double p         Proportional parameter
     double i         Integral parameter
     double d         Derivative parameter
  */
  myPID(double p, double i, double d)
        : input(0.0), output(0.0), setpoint(0.0), kp(p), ki(i), kd(d),
	  PID(&input, &output, &setpoint, p, i, d, DIRECT){}

  /* myPID::init():
     Initializes the myPID object with the following settings:
nnn     mode              AUTOMATIC
     sample time       10 ms
     output limits     -255 -- 255

     Note that this function must be invoked before using the PID.

     @params
     void

     @return
     void
  */
  void init();

  /* myPID::compute():
     Computes the new (adjusted*) motor speed from the PID algorithm given
     the latest angle reading.
     (* See: myPID::updateHist())

     @params
     double inp             The latest angle reading with which to compute

     @return
     double                 The latest motor output result of the computation
  */
  double compute(double inp);
  
  /* myPID::updateHist():
     The PID's output is smoothed using a history of previous output values.
     Each new output of the PID is added to the history (pushing out the
     oldest value) and an adjusted output is obtained by calculating the
     arithmetic mean of all entries in the history. This adjustment produces
     a less erratic and 'smoother', albeit slower responding, output.
     This is desirable when controlling highly responsive motors (as in this
     application).
     (See: myPID::histLen)

     @params
     double output         The most recent raw output value from the PID

     @return
     double                The history mean after adding the new value
  */ 
  double updateHist(double output);

  /* myPID::setTunings():
     Sets the three tuning variables (P, I, D) to the given values.

     @params
     double p              The desired P value
     double i              The desired I value
     double d              The desired D value

     @return
     void
  */
  void setTunings(double p, double i, double d){
    this -> SetTunings(kp = p, ki = i, kd = d);
  }

  // myPID::getKp(): Returns the current P value
  double getKp(){ return kp; }

  // myPID::getKi(): Returns the current I value
  double getKi(){ return ki; }

  // myPID::getKd(): Returns the current D value
  double getKd(){ return kd; }

  // myPID::getSetpoint(): Returns the current setpoint
  double getSetpoint(){ return setpoint; }

  // myPID::getInput(): Returns the current input
  double getInput(){ return input; }

  // myPID::getOutput(): Returns the current output
  double getOutput(){ return output; }

  // myPID::setSetpoint(): Sets the setpoint to the given double
  void setSetpoint(double val){ setpoint = val; }

  // myPID::setInput(): Sets the input to the given double
  void setInput(double val){ input = val; }

  // myPID::setOutput(): Sets the output to the given double
  void setOutput(double val){ output = val; }

  /* myPID::reset():
     Resets the PID, erasing its history (both output and integral)
     and putting it into the same state as it had been before being run.

     @params
     void

     @return
     void
  */
  void reset();
  
private:
  double setpoint, input, output, kp, ki, kd;
  static const int histLen = 9; // Number of entries in output history
  double hist[histLen];
};
// End class myPID

#endif
