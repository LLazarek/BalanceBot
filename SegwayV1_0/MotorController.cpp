#include <SPI.h>
#include "MotorController.h"
#include "serialpp.h"

// The DACs don't output exactly what they're given
// but 2.5v exactly is needed for RobotEQ initialization
#define two_p_five 2.54 // This value outputs 2.5v from DACs
#define DAC1 10         // Pin for DAC1
#define DAC2 8          // Pin for DAC2


void MotorController::boot(){
  pinMode(DAC1, OUTPUT);
  pinMode(DAC2, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DAC1, HIGH); // default DAC ss pin state
  digitalWrite(DAC2, HIGH);
}

void MotorController::DAC(int dac, double volt){
  unsigned short bitPattern = (int)(volt*4095/5);// 4095/5 per volt
  byte upperByte=highByte(bitPattern); 
  byte lowerByte=lowByte(bitPattern);
  upperByte |=0x30;// 3 = Control Nibble (4 bits) | 0 = 2nd Part of MSB (4-bits)
  digitalWrite(dac, LOW);
  SPI.transfer(upperByte);
  SPI.transfer(lowerByte);
  digitalWrite(dac, HIGH);
}

void MotorController::init(){
  DAC(DAC1, 2.4); // Apply voltage that is NOT 2.5v to init analog connection
  DAC(DAC2, 2.4);
  delay(1000);
  DAC(DAC1, two_p_five); // Apply sustained 2.5v to activate analog control
  DAC(DAC2, two_p_five);
  delay(1000);
}

void MotorController::write(int spd){
  spd = constrain(spd, -250, 250);
  spd += 250;
  DAC(DAC1, 5.0 - ((double)spd*DAC1_mult)/100);// Polarity of one motor flipped
  DAC(DAC2, ((double)spd*DAC2_mult)/100);
}
