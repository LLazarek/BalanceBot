import processing.serial.*;
import controlP5.*;

ControlP5 cp5;// GUI library object for textbox

// Serial Communication variables
Serial myPort;
String inString;

// Line drawing variables (keep track of line position)
int xPos = 1;
int xPos2 = 1;
int lastxPos = 1;
int lastxPos2 = 1;
int lastheight = 0;
int lastheight2 = 0;

// PID tuning variables (keep track of tunings on Arduino)
float kp = 0, ki = 0, kd = 0;

/* setup:
   Initialization function automatically called at program start
*/
void setup () {
  size(1300, 600);// Window size

  myPort = new Serial(this, "/dev/ttyACM0", 115200);
  myPort.bufferUntil('\n');// Call serialEvent() when newline read by serial
  
  resetScrn();// Initialize window
  
  //Text box stuff
  PFont font = createFont("arial",10);
  cp5 = new ControlP5(this);
  cp5.addTextfield("input")
     .setPosition(10,10)
     .setSize(200,25)
     .setFont(font)
     .setFocus(true)
     .setColor(color(254,254,254));
   textFont(font);
}

/* input:
   An event that is automatically called when textbox "input" registers
   the enter key

   1. Sends textbox string to Arduino via Serial
   2. Updates local tunings if the string contains relevant info
*/
public void input(String theText) {
  myPort.write(theText);
  updateK(theText);
}

/* draw:
   Automatically called after serialEvent()
   Processes Arduino serial data

   1. Updates graphs with new data if available
   2. Otherwise processes non-data communications
*/
void draw () {
  if (inString != null) {
    if(inString.contains("\t")){// Only graphing data contains tab
      String[] strs = split(inString, '\t');
      updateLn1(strs[0]);
      updateLn2(strs[1]);
    }else{                      // All other serial output from Arduino
      println(inString);
      if(inString.contains("Stabilize")) resetGraph();// Arduino restarted
      else if(inString.contains("K vals: ")) updateK(inString);
    }
  }
}

/* serialEvent:
   Automatically called when serial buffer registers newline character
   (see setup())
   Stores read string for processing in draw()
*/
void serialEvent (Serial myPort) {
  inString = myPort.readStringUntil('\n');// Retrieve serial data from buffer
}

/* getNum:
   Searches a given string for a number and converts it to a float if found

   @params
   str:            The string containing a number

   @return
   -1 or NaN       if no number found in string
   value read      if value successfully extracted
*/
float getNum(String str){
  int c, l = str.length();
  for(int i = 0; i < l; i++){
      c = str.charAt(i);
      if((c <= 57 && c >= 48) || c == 46){// c is a digit or '.'
        return float(str.substring(i, l));
      }
  }
  return -1;
}

/* updateK:
   Updates local storage of current PID tunings if present in given string

   @params
   str:            The string (possibly) containing pid tuning values
*/
void updateK(String str){
  String parts[] = str.split(",");
  for(int i = 0; i < parts.length; i++){
    if(parts[i].contains("p")){
      kp = getNum(parts[i]);
    }else if(parts[i].contains("i")){
      ki = getNum(parts[i]);
    }else if(parts[i].contains("d")){
      kd = getNum(parts[i]);
    }
  }
  println("K vals: kp = " + kp + ", ki = " + ki + ", kd = " + kd);
}

/* resetScrn:
   Clears the window before redrawing graph axes and labels
*/
void resetScrn(){
  background(254, 254, 254); // Clear screen
  
  //Draw x-axis
  stroke(1, 1, 1);
  line(0, height/2, width, height/2);

  //Create scale markings:
  float h = height/10;
  float w = width - 20;
  textSize(10);
  fill(0, 0, 255);    // Blue markings
  text("1", 0, 4*h);
  text("2", 0, 3*h);
  text("3", 0, 2*h);
  text("4", 0, h);
  text("-1", 0, 6*h);
  text("-2", 0, 7*h);
  text("-3", 0, 8*h);
  text("-4", 0, 9*h);
  fill(255, 0, 0);    // Red markings
  text("51", w - 10, 4*h);
  text("102", w - 10, 3*h);
  text("153", w - 10, 2*h);
  text("204", w - 10, h);
  text("-51", w - 10, 6*h);
  text("-102", w - 10, 7*h);
  text("-153", w - 10, 8*h);
  text("-204", w - 10, 9*h);
  
  // Draw scale lines
  stroke(200, 200, 200);
  line(0, 4*h, width, 4*h);
  line(0, 3*h, width, 3*h);
  line(0, 2*h, width, 2*h);
  line(0, h, width, h);
  line(0, 6*h, width, 6*h);
  line(0, 7*h, width, 7*h);
  line(0, 8*h, width, 8*h);
  line(0, 9*h, width, 9*h);
}

/* resetGraph:
   Resets line graphing variables to starting values and clears the graph
   (See: resetScrn())
*/
void resetGraph(){
  xPos = 0;
  xPos2 = 0;
  lastxPos = 0;
  lastxPos2 = 0;
  resetScrn();
}

/* updateLn1:
   Updates the plot of the blue line with the new value read by Serial
   Resets the graph when the line reaches the end of window

   @params
   val:        The new value to be appended to the graph
*/
void updateLn1(String val){
  val = trim(val);
  float readVal = float(val);
  readVal = map(readVal, -5, 5, 0, height);// Map from value scale to screen size

  stroke(0,0,255); // Blue line
  strokeWeight(2);
  line(lastxPos, lastheight, xPos, height - readVal); 
  lastxPos = xPos;
  lastheight = int(height - readVal);

  if(xPos >= width) {  // At the edge of window
      resetGraph();
  }else{
      xPos++; // Continue line
  }
}

/* updateLn2:
   Updates the plot of the red line with the new value read by Serial

   @params
   val:        The new value to be appended to the graph
*/
void updateLn2(String val){
  val = trim(val);
  float readVal = float(val);
  readVal = map(readVal, -255, 255, 0, height);// Map from value scale to screen size

  stroke(255,0,0);     // Red line
  strokeWeight(2);
  line(lastxPos2, lastheight2, xPos2, height - readVal); 
  lastxPos2 = xPos2;
  lastheight2 = int(height - readVal);

  // Graph resetting handled by updateLn1()
  xPos2++;
}
