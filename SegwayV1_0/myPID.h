#ifndef MYPID_H
#define MYPID_H

#include <PID_v1.h>

// Begin class myPID
class myPID : public PID {
public:
  myPID(double p, double i, double d) : input(0.0), output(0.0), setpoint(0.0), kp(p), ki(i), kd(d), PID(&input, &output, &setpoint, p, i, d, DIRECT) {}
  
  void init();
  double updateHist(double output);
  
  void setTunings(double p, double i, double d){
    this -> SetTunings(kp = p, ki = i, kd = d);
  }
  
  double getKp(){ return kp; }
  double getKi(){ return ki; }
  double getKd(){ return kd; }

  double getSetpoint(){ return setpoint; }
  double getInput(){ return input; }
  double getOutput(){ return output; }
  void setSetpoint(double val){ setpoint = val; }
  void setInput(double val){ input = val; }
  void setOutput(double val){ output = val; }

  void reset();
  
private:
  double setpoint, input, output, kp, ki, kd;
  static const int histLen = 9;
  double hist[histLen];
};
// End class myPID

#endif
