#include "mystd.h"

#define accel_y_pin A0
#define accel_sensitivity 1/17.28 // Accelerometer sensitivity
// 0.00488758553274682/0.0173
#define gyro_conversion_DPS 8.75/1000 // Conversion angular velocity
// 8.75/1000
#define loop_time 10 // in ms
#define sampleNum 20 // Number of samples for bias calculation
#define LEDpin 2
#define switchPin 7
#define histLen 9 // Number of entries to use in rolling average
#define DAC1 10
#define DAC2 8
#define two_p_five 2.54 // The DACs don't quite output 2.5V exactly - use this val to init

#define PRINT_STATUS  Print5("K vals: kp = ", kp, ", ki = ", ki, ", kd = ", kd, "\nleft = ", DAC1_mult, ", right = ", DAC2_mult, "\n")

L3G gyro;
LSM303 compass;

// Angle detection variables
double accel_bias;
double gyro_bias;
double filtered_angle = 0;
int fallen = 0;// Flag for fall detection

double hist[histLen];// History for rolling average of PID output

// PID
double setpoint, input, output;
double kp = 10, ki = 0, kd = 1; // PID tuning values
//150, 2000, 8
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT); // PID object

double DAC1_mult = 1.0, DAC2_mult = 1.0;
bool kill = 0;

/************************** Begin Functions ************************/

/* PID_init:
   Initializes and starts the PID controller

   @params
   void

   @return
   void
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

   @params
   void

   @return
   void
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
  Print1("Gyro Bias: ", gyro_bias, "");
}

/* accel_calcBias:
   Calculates the accelerometer bias
   (See: sampleNum)

   @params
   void

   @return
   void
*/
void accel_calcBias(){
  accel_bias = 0;
  
  for(int n = 0; n < sampleNum; n++)
  {
    compass.read();
    accel_bias += compass.a.z >> 4;// last 4 bits are 0
    delay(50);
  }
  accel_bias = accel_bias/sampleNum;
  Print1("\tAccel Bias: ", accel_bias, "\n");
}

/* biasInit:  
   Performs bias initialization for gyro and accel.
   (See: gyro_calcBias(), accel_calcBias())

   @params
   void

   @return
   void
*/
void biasInit(){
  Println("Beginning bias initialization...");
  gyro_calcBias();
  accel_calcBias();
  Println("Initialization complete.");
}


/* gyro_readRate:
   Reads gyro angular velocity data
   (See: gyro_bias, gyro_calcBias(), gyro_conversion_DPS)

   @params
   void

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

   @params
   void

   @return
   The approximate angle reading
*/
double accel_readAngle(){
  static double accel_y;

  compass.read();
  accel_y = compass.a.z >> 4;
  return (accel_y - accel_bias)*accel_sensitivity;
}

/* filter:
   Applys Complementary filter to the sensors to create an estimated angle

   @params
   gyro_rate:     The current angular rate of change (read by the gyro)
   accel_angle:   The current angle reading (read by the accelerometer)
  
   @return
   The filtered angle output
*/
double filter(double gyro_rate, double accel_angle){
  filtered_angle = (gyro_rate*loop_time/1000 + filtered_angle)*0.97 + accel_angle*0.03;
  return filtered_angle;
}

/* DAC:
   Controls one Digital to Analog Converter to ouput the given voltage analog signal
   using SPI to communicate with the DAC.

   @params
   dac:            The DAC chip to control
   volt:           The voltage analog signal to be output

   @return
   void
*/
void DAC(int dac, float volt)
{
  unsigned short bitPattern = (int)(volt*4095/5);// 4095/5 per volt
  byte upperByte=highByte(bitPattern); 
  byte lowerByte=lowByte(bitPattern);
  upperByte |=0x30;//the 3 is for control | 0x30 = HEX | 3 = Control Nibble (4-bits) | 0 = 2nd Part of MSB Nibble (4-bits) -> 1 byte
  digitalWrite(dac, LOW);
  SPI.transfer(upperByte);
  SPI.transfer(lowerByte);
  digitalWrite(dac, HIGH);
}

/* motorControl:
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
void motorControl(int spd){
  spd = constrain(spd, -250, 250);
  spd += 250;
  DAC(DAC1, 5.0 - ((float)spd*DAC1_mult)/100);// Polarity of one motor is flipped
  DAC(DAC2, ((float)spd*DAC2_mult)/100);
}

/* robotEQ_init:
   Ensures that the RobotEQ motor controller is initialized to Analog Control Mode with
   a constant 2.5V signal for 1 sec. 

   @params
   void

   @return
   void
*/
void robotEQ_init(){
  DAC(DAC1, 2.4);
  DAC(DAC2, 2.4);
  delay(1000);
  DAC(DAC1, two_p_five);
  DAC(DAC2, two_p_five);
  delay(1000);
}

/* updateTunings:
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
void updateTunings(){
  char var;
  double res;
  
  var = Serial_ReadChar();// See std.h
  if(var != -1 && var == 's'){
    Print("Emergency software kill/override activated.\n");
    kill = true;
  }else if(var != -1 && (var == 'p' || var == 'i' || var == 'd' || var == 'l' || var == 'r')){
    res = Serial_ReadFloat();// See std.h
    Serial_RmWhiteSpc();// See std.h
    
    if(var == 'p'){
      Print2("Changed kp value from ", kp, " to ", res, "\n");
      kp = res;
    }else if(var == 'i'){
      Print2("Changed ki value from ", ki, " to ", res, "\n");
      ki = res;
    }else if(var == 'd'){
      Print2("Changed kd value from ", kd, " to ", res, "\n");
      kd = res;
    }else if(var == 'l'){
      Print2("Changed left wheel mult from ", DAC1_mult, " to ", res, "\n");
      DAC1_mult = res;
    }else if(var == 'r'){
      Print2("Changed right wheel mult from ", DAC2_mult, " to ", res, "\n");
      DAC2_mult = res;
    }
    myPID.SetTunings(kp, ki, kd);
    PRINT_STATUS;
  }else if(var == '.'){
    PRINT_STATUS;
  }
}

/* reset:
   Re-initializes all necessary variables etc to restart the
   balancing algorithm

   @params
   void

   @return
   void
*/
void reset(){
  filtered_angle = 0;
  for(int i = 0; i < histLen; i++) hist[i] = 0;
  fallen = 0;
  biasInit();

  /* Reset PID */
  myPID.SetMode(MANUAL);
  // Force PID I-term and output to 0
  myPID.SetOutputLimits(0.0, 1.0);// Force min to 0
  myPID.SetOutputLimits(-1.0, 0.0);// Force max to 0
  PID_init();

  //Re-init RobotEQ
  robotEQ_init();
}

