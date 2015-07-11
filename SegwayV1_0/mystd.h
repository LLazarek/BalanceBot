#include <Arduino.h>

/* Printing definitions, intended to make printing combinations of strings
   and values to the Serial moniter less painful and more concise. */

// Print: Prints the given string to the serial moniter
#define Print(x) Serial.print(x)

// Println: Prints the given string to the serial moniter, appending a nl
#define Println(x) Serial.println(x)

/* Print1: Prints the given string string1, followed by the given value a,
           followed by the second string string2 */
#define Print1(str1, a, str2) Print(str1), Print(a), Print(str2)

// Print2: Extension of Print1 for 3 strings and 2 values
#define Print2(str1, a, str2, b, str3) Print1(str1, a, str2), Print(b),	\
    Print(str3)

// Print3: Extension of Print1 for 4 strings and 3 values
#define Print3(str1, a, str2, b, str3, c, str4)		\
  Print2(str1, a, str2, b, str3), Print(c), Print(str4)

// Print4: Extension of Print1 for 5 strings and 4 values
#define Print4(str1, a, str2, b, str3, c, str4, d, str5)		\
  Print3(str1, a, str2, b, str3, c, str4), Print(d), Print(str5)

// Print5: Extension of Print1 for 6 strings and 5 values
#define Print5(str1, a, str2, b, str3, c, str4, d, str5, e, str6)	\
  Print4(str1, a, str2, b, str3, c, str4, d, str5), Print(e), Print(str6)

// Print6: Extension of Print1 for 7 strings and 6 values
#define Print6(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7) \
  Print5(str1, a, str2, b, str3, c, str4, d, str5, e, str6), Print(f),\
  Print(str7)

// Print7: Extention of Print1 for 8 strings and 7 values
#define Print7(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, \
	       g, str8) \
  Print6(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7), \
  Print(g), Print(str8)

// Print8: Extension of Print1 for 9 strings and 8 values
#define Print8(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, \
	       g, str8, h, str9) \
  Print7(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, g, \
	 str8), Print(h), Print(str9)

// Print9: Extension of Print1 for 10 strings and 9 values
#define Print9(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, \
	       g, str8, h, str9, i, str10) \
  Print8(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, g,\
	 str8, h, str9), Print(i), Print(str10)

// Print10: Extension of Print1 for 11 strings and 10 values
#define Print10(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7,\
		g, str8, h, str9, i, str10, j, str11) \
  Print9(str1, a, str2, b, str3, c, str4, d, str5, e, str6, f, str7, g,\
	 str8, h, str9, i, str10), Print(j), Print(str11)

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
