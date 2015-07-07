#include <Arduino.h>

/* Printing definitions, intended to make printing combinations of strings
   and values to the Serial moniter less painful and more concise. */

// print: prints the given string to the serial moniter
#define print(x) Serial.print(x)

// println: prints the given string to the serial moniter, appending a nl
#define println(x) Serial.println(x)

/* print1: prints the given string string1, followed by the given value a,
           followed by the second string string2 */
#define print1(str1, a, str2) print(str1), print(a), print(str2)

// print2: Extension of print1 for 3 strings and 2 values
#define print2(str1, a, str2, b, str3) print1(str1, a, str2), print(b),	\
    print(str3)

// print3: Extension of print1 for 4 strings and 3 values
#define print3(str1, a, str2, b, str3, c, str4)		\
  print2(str1, a, str2, b, str3), print(c), print(str4)

// print4: Extension of print1 for 5 strings and 4 values
#define print4(str1, a, str2, b, str3, c, str4, d, str5)		\
  print3(str1, a, str2, b, str3, c, str4), print(d), print(str5)

// print5: Extension of print1 for 6 strings and 5 values
#define print5(str1, a, str2, b, str3, c, str4, d, str5, e, str6)	\
  print4(str1, a, str2, b, str3, c, str4, d, str5), print(e), print(str6)

// print6: Extension of print1 for 7 strings and 6 values
#define print6(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7) \
  print5(str1, a, str2, b, str3, c, str4, d, str5, e, str6), print(f),\
  print(str7)

// print7: Extention of print1 for 8 strings and 7 values
#define print7(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, \
	       g, str8) \
  print6(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7), \
  print(g), print(str8)

// print8: Extension of print1 for 9 strings and 8 values
#define print8(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, \
	       g, str8, h, str9) \
  print7(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, g, \
	 str8), print(h), print(str9)

// print9: Extension of print1 for 10 strings and 9 values
#define print9(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, \
	       g, str8, h, str9, i, str10) \
  print8(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, g,\
	 str8, h, str9), print(i), print(str10)

// print10: Extension of print1 for 11 strings and 10 values
#define print10(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7,\
		g, str8, h, str9, i, str10, j, str11) \
  print9(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, g,\
	 str8, h, str9, i, str10), print(j), print(str11)

/*************************************************************************/
/* Character interpretation definitions */

// charIsDigit: Determines if the given character is a digit(0-9)
#define charIsDigit(c) (c >= 48 && c <= 57)

// chPartofNum: Determines if the given character is either a digit or '.'
#define chPartofNum(c) (charIsDigit(c) || c == '.')

/*************************************************************************/
/* Serial reading functions */

/* checkSerialMon:
   Checks if there is data in the Serial buffer

   @return
   0        if no data available
   # > 0    if data available
*/
int checkSerialMon(void){
  return Serial.available();
}

/* Serial_RmWhiteSpc:
   Removes white space from the Serial buffer
*/
void Serial_RmWhiteSpc(void){
  if(Serial.available() <= 0) return;
  
  char c = Serial.peek();
  if(c == 13 || c == 10 || c == ' '){
    Serial.read();
    Serial_RmWhiteSpc();
  }
}

/* Serial_ReadChar:
   Reads a character from the Serial moniter

   @return:
   -1                if no char could be read
   char value        char successfully read
*/
char Serial_ReadChar(void){
  if(Serial.available() > 0) return Serial.read();
  else return -1;
}

/* Serial_ReadInt:
   Reads an integer from the Serial moniter

   @ return:
   -9999             if no int could be read
   int value         int successfully read
*/
int Serial_ReadInt(void){
  if(Serial.available() > 0) return Serial.parseInt();
  else return -9999;
}

/* Serial_ReadFloat:
   Reads a float/double from the Serial moniter (Max 14 digits)

   @ return:
   -9999.99          if no float could be read
   float value       float successfully read
*/
float Serial_ReadFloat(void){
  char arr[15], c;
  if(Serial.available() > 0){
    c = Serial.peek();
    // Remove spaces, if any
    if(c == ' ') Serial_RmWhiteSpc(), c = Serial.peek();
    
    for(int i = 0; i < 15 && chPartofNum(c); i++){
      c = Serial.read();
      arr[i] = c;
      c = Serial.peek();
    }
    return atof(arr);
  }else return -9999.99;
}

/**************************************************************************/
/* Useful Arduino functions */

/* software_Reset:
   Restarts the arduino sketch
*/
void (* software_Reset) (void) = 0;

/*************************************************************************/
