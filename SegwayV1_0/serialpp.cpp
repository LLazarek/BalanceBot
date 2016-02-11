#include "serialpp.h"

int serialpp::inputAvailable(void){
  return Serial.available();
}

void serialpp::rmWhiteSpc(void){
  if(Serial.available() <= 0) return;
  
  char c = Serial.peek();
  if(c == 13 || c == 10 || c == ' '){
    Serial.read();
    rmWhiteSpc();
  }
}

char serialpp::readChar(void){
  if(Serial.available() > 0) return Serial.read();
  else return -1;
}

int serialpp::readInt(void){
  if(Serial.available() > 0) return Serial.parseInt();
  else return -9999;
}

double serialpp::readDouble(void){
  char arr[15], c;
  if(Serial.available() > 0){
    c = Serial.peek();
    // Remove spaces, if any
    if(c == ' ') rmWhiteSpc(), c = Serial.peek();
    
    for(int i = 0; i < 15 && chPartofNum(c); i++){
      c = Serial.read();
      arr[i] = c;
      c = Serial.peek();
    }
    return atof(arr);
  }else return -9999.99;
}
