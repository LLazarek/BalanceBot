/* Test basic angle detection with ADXL203EB
   Accelerometer. Data sheet:
   http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL103_203.pdf
*/

#define yPin A0

int baseline;

void setup(){
  pinMode(yPin, INPUT);
  Serial.begin(9600);
  
  int sum = 0;
  for(int i = 0; i < 10; i++){
    sum += analogRead(yPin);
  }
  baseline = sum/10;
}

void loop(){
  int y = analogRead(yPin);
  Serial.print(" - Yraw: ");
  Serial.print(y);
  Serial.print(" - yRel: ");
  Serial.println((y - baseline)/3.5);
  
  
  delay(100);
}
