/* Angle detection with Pololu Alt-IMUv4 L3GD20H
   gyroscope. Data sheet:
   https://www.pololu.com/file/0J731/L3GD20H.pdf
*/
/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/

#include <Wire.h>
#include <L3G.h>

#define CONVN_DPS 8.75/1000
#define checkRate 10 /* The rate at which the sensor checks its position, in milliseconds */

L3G gyro;
float xAngular, yAngular, zAngular;
float yAngle = 0;

float bias;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
  
  /* Initialize baseline bias value */
  float sum = 0;
  for(int i = 0; i < 10; i++){
    gyro.read();
    sum += gyro.g.y;
    delay(100);
  }
  bias = sum/10;

  Serial.println(bias), Serial.println();
}

void loop() {
  gyro.read();

  /*Serial.print("G ");
  Serial.print("X: ");
  xAngular = gyro.g.x * CONVN_DPS + 1.85;
  Serial.print(xAngular);*/
  
  /* Calculate angular velocity using conversion and base bias */
  yAngular = (gyro.g.y - bias) * CONVN_DPS;
  //yAngular = ((int)(yAngular*10))/10;
  //Serial.print("  Y_AngularVelocity: ");
  Serial.print(yAngular);
  
  
  /*Serial.print(" Z: ");
  zAngular = gyro.g.z * 8.75/1000 - 0.22;
  Serial.print(zAngular);*/
  
  yAngle = yAngle + yAngular * checkRate/1000;

  Serial.print("\t");
  Serial.println(yAngle);

  delay(checkRate);
}
/*
float average(float nums[], int n){
  float res = 0;
  for(int i = 0; i < n; i++){
    res += nums[i];
  }
  
  return res/n;
}*/
