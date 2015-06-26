import processing.serial.*;
import controlP5.*;

ControlP5 cp5;
public float numberboxValue = 0;

String inString;

Serial myPort;        // The serial port
int xPos = 1;         // horizontal position of the graph 
int xPos2 = 1;

//Variables to draw a continuous line.
int lastxPos=1;
int lastheight=0;
int lastxPos2 = 1;
int lastheight2 = 0;

float kp = 0, ki = 0, kd = 0;

void setup () {
  // set the window size:
  size(1300, 600);

  myPort = new Serial(this, "/dev/ttyACM0", 115200);  //

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  resetScrn();
  
  //Text box stuff
  PFont font = createFont("arial",10);
  cp5 = new ControlP5(this);
  cp5.addTextfield("input")
     .setPosition(10,10)
     .setSize(200,25)
     .setFont(font)
     .setFocus(true)
     .setColor(color(254,254,254))
     ;
     
  textFont(font);
}

public void input(String theText) {
  // automatically receives results from controller input
  interpret(theText);
}

void interpret(String str){
  myPort.write(str);
  updateK(str);
}

void draw () {
  if (inString != null) {
    if(inString.contains("\t")){
      String[] strs = split(inString, '\t');
      
      // Process first #
      
      updateLn1(strs[0]);
      
      updateLn2(strs[1]);
    }else{
      println(inString);
      if(inString.contains("Stabilize")) resetGraph();
      else if(inString.contains("K vals: ")) updateK(inString);
    }
  }
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  inString = myPort.readStringUntil('\n');
}

float getNum(String str){
  int c, l = str.length();
  for(int i = 0; i < l; i++){
      c = str.charAt(i);
      if((c <= 57 && c >= 48) || c == 46){
        return float(str.substring(i, l));
      }
  }
  return -1;
}

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

void resetScrn(){
  //Clear screen
  background(254, 254, 254);
  
  //Draw x-axis
  stroke(1, 1, 1);
  line(0, height/2, width, height/2);
  
  float h = height/10;
  float w = width - 20;
  //Create scale markings:
  fill(0, 0, 255);
  textSize(10);
  text("1", 0, 4*h);
  text("2", 0, 3*h);
  text("3", 0, 2*h);
  text("4", 0, h);
  text("-1", 0, 6*h);
  text("-2", 0, 7*h);
  text("-3", 0, 8*h);
  text("-4", 0, 9*h);
  fill(255, 0, 0);
  text("51", w - 10, 4*h);
  text("102", w - 10, 3*h);
  text("153", w - 10, 2*h);
  text("204", w - 10, h);
  text("-51", w - 10, 6*h);
  text("-102", w - 10, 7*h);
  text("-153", w - 10, 8*h);
  text("-204", w - 10, 9*h);
  
  
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

void resetGraph(){
  xPos = 0;
  lastxPos = 0;
  xPos2 = 0;
  lastxPos2 = 0;
  resetScrn();
}

void updateLn1(String val){
  val = trim(val);                // trim off whitespaces.
  float inByte = float(val);           // convert to a number.
  inByte = map(inByte, -5, 5, 0, height); //map to the screen height.

  //Drawing a line from Last inByte to the new one.
  stroke(0,0,255);     //stroke color
  strokeWeight(2);        //stroke wider
  line(lastxPos, lastheight, xPos, height - inByte); 
  lastxPos= xPos;
  lastheight= int(height-inByte);

  // at the edge of the window, go back to the beginning:
  if (xPos >= width) {
    resetGraph();
  } 
  else {
    // increment the horizontal position:
    xPos++;
  }
}

void updateLn2(String val){
  val = trim(val);                // trim off whitespaces.
  float inByte = float(val);           // convert to a number.
  inByte = map(inByte, -255, 255, 0, height); //map to the screen height.

  //Drawing a line from Last inByte to the new one.
  stroke(255,0,0);     //stroke color
  strokeWeight(2);        //stroke wider
  line(lastxPos2, lastheight2, xPos2, height - inByte); 
  lastxPos2 = xPos2;
  lastheight2 = int(height-inByte);

  xPos2++;
}
