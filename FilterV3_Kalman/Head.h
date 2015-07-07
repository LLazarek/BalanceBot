#include "mystd.h"

#define accel_y_pin A0
#define accel_sensitivity 0.282519395 // Accelerometer sensitivity
// 0.00488758553274682/0.0173
#define gyro_conversion_DPS 0.00875 // Conversion to angular velocity
// 8.75/1000
#define loop_time 10 // in ms
#define sampleNum 20 // Number of samples for bias calculation
#define LEDpin 2
#define switchPin 7
#define histLen 9 // Number of entries to use in rolling average

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor_1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor_2 = AFMS.getMotor(2);

L3G gyro;

// Angle detection variables
double accel_bias = 0;
double gyro_bias = 0;
double filtered_angle = 0;
int fallen = 0;// Flag for fall detection

double hist[histLen];// History for rolling average of PID output

// PID
double setpoint, input, output;
double kp = 55, ki = 2000, kd = 9;
// Successful tuning values: 55, 2000, 9 (kp 100), (ki 1000-2000)
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

/************************** Kalman Filter Vars *********************/
float Q_angle2 = 0.001;
float Q_gyroBias = 0.003;
float R_measure = 0.03;
float angle = 0, rate = 0;
float rate_bias = 0;
float P[2][2];

/************************** Begin Functions ************************/

/* PID_init:
   Initializes and starts the PID controller
*/
void PID_init(){
  setpoint = 0;
  input = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10); // 10 ms loop
  myPID.SetOutputLimits(-255,255);
}

/* PID_Hist:
   Calculates a rolling average of PID output, smoothing its response
   (See: histLen)

   @params
   output:     The most recent PID output value

   @return
   The average of the updated history (including most recent value)
*/
double PID_Hist(double output){
  static int h = 0;
  double sum = 0;
  
  hist[h] = output;
  h = (h + 1)%histLen;
  
  for(int i = 0; i < histLen; i++){
    sum = sum + hist[i];
  }
  return sum/histLen;
}


/* gyro_calcBias:
   Calculates the gyroscope bias
   (See: sampleNum)
*/
void gyro_calcBias(){
  gyro_bias = 0;
  
  for(int n = 0; n < sampleNum; n++)
  {
    gyro.read();
    gyro_bias += gyro.g.y;
    delay(50);
  }
  gyro_bias = gyro_bias/sampleNum;
  print1("Gyro Bias: ", gyro_bias, "");
}

/* accel_calcBias:
   Calculates the accelerometer bias
   (See: sampleNum)
*/
void accel_calcBias(){
  accel_bias = 0;
  
  for(int n = 0; n < sampleNum; n++)
  {
    accel_bias += analogRead(accel_y_pin);
    delay(50);
  }
  accel_bias = accel_bias/sampleNum;
  print1("\tAccel Bias: ", accel_bias, "\n");
}

// Wrapper function for bias initialization
void biasInit(){
  println("Beginning bias initialization...");
  gyro_calcBias();
  accel_calcBias();
  println("Initialization complete.");
}


/* gyro_readRate:
   Reads gyro angular velocity data
   (See: gyro_bias, gyro_calcBias(), gyro_conversion_DPS)

   @return
   The approximate angular rate of change
*/
double gyro_readRate(){
  static double gyro_velocity;
  
  gyro.read();
  gyro_velocity = (gyro.g.y - gyro_bias) * gyro_conversion_DPS;
  return gyro_velocity;
}

/* accel_readAngle:
   Reads accelerometer data and converts it into an approximate angle
   (See: accel_bias, accel_calcBias(), accel_sensitivity)

   @return
   The approximate angle reading
*/
double accel_readAngle(){
  static double accel_y;
  
  accel_y = analogRead(accel_y_pin);
  return (accel_y - accel_bias)*accel_sensitivity*2;
  // Accel is reading ~1/2 the actual angle, so mult by 2
}


/* kalman:
   Applys Kalman filter to produce filtered angle estimate from accel
   and gyro data

   @params
   newAngle      The angle read by the accelerometer
   newRate       The angular velocity read by the gyroscope

   @return
   The filtered angle estimate

   Credits: Code (with slight modification) obtained from TKJ Electronics blog
   http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
*/
float kalman(float newAngle, float newRate){
  float K[2];
  float dt = float(loop_time)/1000;
  
  angle += dt * (newRate - rate_bias);
  
  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle2);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyroBias * dt;
  
  float y = newAngle - angle;
  float S = P[0][0] + R_measure;
  
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  
  angle += K[0] * y;
  rate_bias += K[1] * y;
  
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  
  return angle;
}

/* motorControl:
   Controls the motors
  
   @params
   spd:       The speed for the motors (in range (-255, 255), signed)
              indicating direction of rotation.
   -255 - -1    = Backward
   0            = Stop
   1 - 255      = Forward
*/
void motorControl(int spd){
  motor_1 -> run(spd > 0 ? BACKWARD : FORWARD);
  motor_2 -> run(spd > 0 ? BACKWARD : FORWARD);
  
  motor_1 -> setSpeed(spd = abs(spd));
  motor_2 -> setSpeed(spd);
}

/* updateTunings:
   Reads data from the Serial moniter and interprets commands to update
   PID tuning variables during runtime

   Valid input format: "kp 55.6", "kp53",  "p 53", "p55.6", "ki 53"...
   Invalid input is thrown away
*/
void updateTunings(void){
  char var;
  double res;
  
  var = Serial_ReadChar();// See std.h
  if(var != -1 && (var == 'p' || var == 'i' || var == 'd')){
    res = Serial_ReadFloat();// See std.h
    Serial_RmWhiteSpc();// See std.h
    
    if(var == 'p'){
      print2("Changed kp value from ", kp, " to ", res, "\n");
      kp = res;
    }else if(var == 'i'){
      print2("Changed ki value from ", ki, " to ", res, "\n");
      ki = res;
    }if(var == 'd'){
      print2("Changed kd value from ", kd, " to ", res, "\n");
      kd = res;
    }
    myPID.SetTunings(kp, ki, kd);
    print3("K vals: kp = ", kp, ", ki = ", ki, ", kd = ", kd, "\n");
  }else if(var == '.'){
    print3("K vals: kp = ", kp, ", ki = ", ki, ", kd = ", kd, "\n");
  }
}
