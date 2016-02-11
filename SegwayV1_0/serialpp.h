/* Class serialpp (Serial plus plus):
   A wrapper for functions and macros relating to Serial communication with
   the aim of making working with the serial monitor simpler and more concise.
*/

#ifndef SERIALPP_H
#define SERIALPP_H

#include <Arduino.h>


// Print: Prints the given string to the serial moniter
#define Print(x) Serial.print(x)

// Println: Prints the given string to the serial moniter, appending a newline
#define Println(x) Serial.println(x)

/* Print1: Prints the given string string1, followed by the given value a,
           followed by the second string string2

	   == Example use ==
	   Printing a variable value. Instead of:
	     > Serial.print("x is ");
	     > Serial.print(x);
	     > Serial.println(" radians.");
	   You can do:
	     > Print1("x is ", x, " radians.\n");
*/
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

// Begin class serialpp
class serialpp {
public:
/* serialpp::inputAvailable():
   Checks if there is data in the Serial buffer

   @params
   void
   
   @return
   int     0        if no data available
           # > 0    if data available
*/
static int inputAvailable(void);

/* serialpp::rmWhiteSpc():
   Removes white space from the Serial buffer

   @params
   void

   @return
   void
*/
static void rmWhiteSpc(void);

/* serialpp::readChar():
   Reads a character from the Serial moniter

   @params
   void
   
   @return:
   char    -1                if no char could be read
            other            if char successfully read
*/
static char readChar(void);

/* serialpp::readInt():
   Reads an integer from the Serial moniter

   @params
   void
   
   @ return:
   int    -9999             if no int could be read
           other            if int successfully read
*/
static int readInt(void);

/* serialpp::readDouble():
   Reads a double from the Serial moniter (Max 14 digits)

   @params
   void
   
   @ return:
   double   -9999.99          if no double could be read
             other            if double successfully read
*/
static double readDouble(void);
};
// End class serialpp

#endif
