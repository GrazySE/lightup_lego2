// Daniel Shiffman
// All features test

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

/**------------------------------------------------------------//
 //                                                            //
 //  IDK  - Prototype LIGHTUP LEGO2                            //
 //  av: Alrik He    v.0.1.5                                   //
 //  Malmö högskola                                            //
 //                                                            //
 //      2016-12-18    -     2017-##-##                        //
 //                                                            //
 //                                                            //
 //  Used with kinect v.1  &  arduino with RGBled strips       //
 //                                                            //
 //                                                            //
 --------------------------------------------------------------*/

import org.openkinect.freenect.*;
import org.openkinect.processing.*;
import processing.serial.*;

Serial arduinoPort;  
Kinect kinect;

int mx, my,interval=250;
float deg;
String portName="";
boolean ir = false;
boolean colorDepth = false;
boolean mirror = false;
byte gridSize=12;
int rows=20, columns=20, offsetX, offsetY;
boolean toggle, shift;
int [][] averageColor = new int[200][150];
color average = color(255);
int offset=int(gridSize*0.25), senseWidth, senseHeight;
final int leftBound=210, topBound=105, rightBound=250, bottomBound=190;
final String colorName[]={"RED", "ORANGE", "YELLOW", "LIMEGREEN", "GREEN", "BLUE", "PURPLE", "PINK", "RED"};
final int colorThreshold[]={0, 15, 30, 45, 60, 130, 180, 200, 240, 255};
final byte NUM_OF_RGB_LED=40;
long timer=3000;

// Angle for rotation
float a = 0;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];
int min=40, max=1000, pixelSize=gridSize;
final int NUMBER_PIXELS=40; // strip RGBLEDs
final int BAUDRATE= 19200;
void setup() {
  mx=leftBound+1;
  // size(640, 520);
  kinect = new Kinect(this);
  kinect.initDepth();
  kinect.initVideo();
  //kinect.enableIR(ir);
  //kinect.enableColorDepth(colorDepth);

  deg = kinect.getTilt();
  // kinect.tilt(deg);
  averageColor[0][0]=color(255, 255, 0);
  //strokeCap(SQUARE);


  // Rendering in P3D
  size(640, 480, P3D);
  // kinect = new Kinect(this);
  //kinect.initDepth();
  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  try {
    printArray(Serial.list());
    //portName= Serial.list()[0];
    portName ="COM98";
    arduinoPort = new Serial(this, portName, BAUDRATE);
    println("port succeded");
  }
  catch (Exception e) {
    println(e+" port Failed");
  }
  colorMode(HSB);
  //ortho();
  strokeWeight(pixelSize);
  strokeCap(SQUARE);

  senseWidth=width-leftBound-rightBound;
  senseHeight=height-topBound-bottomBound;
}


void draw() {

  if (shift) {
    colorMode(HSB);

    background(0);

    // Get the raw depth as array of integers
    int[] depth = kinect.getRawDepth();

    // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
    int skip = pixelSize;
    strokeWeight(pixelSize);
    pushMatrix();

    fill(255);
    text( depth[mouseX + mouseY*width], mouseX, mouseY);
    // Translate and rotate
    translate(width/2, height/2, 0);
    rotateY(a);


    for (int x = 0; x < kinect.width; x += skip) {
      for (int y = 0; y < kinect.height; y += skip) {
        int offset = x + y*kinect.width;

        // Convert kinect data to world xyz coordinate
        int rawDepth = depth[offset];
        if (min< rawDepth&&rawDepth<max) {
          PVector v = depthToWorld(x, y, 0);

          stroke(rawDepth*10%255, 255, 255);
          pushMatrix();
          // Scale up by 200
          float factor = 460;
          translate(v.x*factor, v.y*factor, factor-v.z*factor);
          // Draw a point
          point(0, 0);
          popMatrix();
        }
      }
    }
    popMatrix();
    if (keyCode==LEFT)
      a += 0.01f;
    if (keyCode==RIGHT)
      a -= 0.01f;
    // Rotate
    //a += 0.005f;
  } else {

    background(0);
    pushMatrix();
    translate(width*.5, height*.5);
    scale(1.15);


    image(kinect.getVideoImage(), -width*.5, -height*.5);
    // Translate and rotate

    // image(kinect.getDepthImage(), 640, 0);
    fill(255);

    popMatrix();
    strokeWeight(1);
    stroke(0, 0, 0);
    for (int i=0; i<width; i+=gridSize)line(i+offsetX, 0+offsetY, i+offsetX, height+offsetY);
    for (int i=0; i<height; i+=gridSize)line(0+offsetX, i+offsetY, width+offsetX, i+offsetY);

    colorMode(RGB);

    loadPixels();
    for (int i=0; i<width; i+=gridSize)for (int j=0; j<height; j+=gridSize) {
      int averageRed[]= new int[5], averageGreen[]=new int[5], averageBlue[]=new int[5];
      averageRed[0]= int(red(get(i+offset, j+offset)));
      averageRed[1]= int(red(get(i+offset, j+gridSize-offset)));
      averageRed[2]= int(red(get(i+gridSize-offset, j+gridSize-offset)));
      averageRed[3]= int(red(get(i+gridSize-offset, j+offset)));
      averageRed[4]= int(red(get(int(i+gridSize*.5), int(j+gridSize*.5))));

      averageGreen[0]= int(green(get(i+offset, j+offset)));
      averageGreen[1]= int(green(get(i+offset, j+gridSize-offset)));
      averageGreen[2]= int(green(get(i+gridSize-offset, j+gridSize-offset)));
      averageGreen[3]= int(green(get(i+gridSize-offset, j+offset)));
      averageGreen[4]= int(green(get(int(i+gridSize*.5), int(j+gridSize*.5))));

      averageBlue[0]= int(blue(get(i+offset, j+offset)));
      averageBlue[1]= int(blue(get(i+offset, j+gridSize-offset)));
      averageBlue[2]= int(blue(get(i+gridSize-offset, j+gridSize-offset)));
      averageBlue[3]= int(blue(get(i+gridSize-offset, j+offset)));
      averageBlue[4]= int(blue(get(int(i+gridSize*.5), int(j+gridSize*.5))));
      int red=0, green=0, blue=0;
      for (int r : averageRed) red+=r;
      for (int g : averageGreen) green+=g;
      for (int b : averageBlue) blue+=b;
      red=int(red/5);
      green=int(green/5);
      blue=int(blue/5);


      color average =color(red, green, blue);


      averageColor[int(i/gridSize)][int(j/gridSize)]=average;
    }


    if (toggle) {
      strokeWeight(1);
      for (int i=0; i<width; i+=gridSize)for (int j=0; j<height; j+=gridSize) {
        fill(averageColor[int(i/gridSize)][int(j/gridSize)]); 
        //point(i+gridSize*.5, j+gridSize*.5);
        rect(i, j, gridSize, gridSize);
      }
    }

    text(
      "Press 'i' to enable/disable between video image and IR image,  " +
      "Press 'c' to enable/disable between color depth and gray scale depth,  " +
      "Press 'm' to enable/diable mirror mode, "+
      "UP and DOWN to tilt camera   " +
      "Framerate: " + int(frameRate), 10, 515);
  }
  displayBounds();
  if(timer+interval<millis()){
  timer=millis();
  for(int i=0; i<16;i++) getColumn(mx+i*gridSize,int(i*2.5));
  }
    key='1';
  //displayRowCapture(mx);
}

void keyPressed() {
  //println(Integer.parseInt(str(key)));
  if (key==' ') {
    arduinoPort.write("1,0xFF0000\0");
    arduinoPort.write("2,0xFFFF00\0");
    arduinoPort.write("3,0x00FF00\0");
    arduinoPort.write("4,0x00FFFF\0");
    arduinoPort.write("5,0x0000FF\0");
    arduinoPort.write("6,0xFF00FF\0");
    background(255);
  }
  if (48<=int(key) && int(key)<=57) {
    arduinoPort.write(Integer.parseInt(str(key)));
    background(255);
  }
  if (key==DELETE) {
    for (int i=0; i<NUM_OF_RGB_LED; i++)arduinoPort.write(i+",0x000000\0");
    //arduinoPort.write("delete ");
    background(255);
  }

  if (key=='s')shift=!shift;
  if (key=='-'&&gridSize>5) {
    gridSize--;
    offset=int(gridSize*0.25);
  }
  if (key=='+'&&gridSize<width) {                       
    gridSize++;
    offset=int(gridSize*0.25);
  }
  if (key== 't') toggle=!toggle;
  if (key == 'i') {
    ir = !ir;
    kinect.enableIR(ir);
  } else if (key == 'c') {
    colorDepth = !colorDepth;
    kinect.enableColorDepth(colorDepth);
  } else if (key == 'm') {
    mirror = !mirror;
    kinect.enableMirror(mirror);
  } else if (key == CODED) {
    if (keyCode == UP) {
      deg++;
    } else if (keyCode == DOWN) {
      deg--;
    }
    deg = constrain(deg, 0, 30);
    kinect.setTilt(deg);
  }

  if (key=='-'&&pixelSize>1)pixelSize--;
  if (key=='+'&&pixelSize<width)pixelSize++;
  if (key=='0')a=0;


  if (keyCode==UP)offsetY++; 
  if (keyCode==DOWN)offsetY--;
  if (keyCode==LEFT)offsetX--;
  if (keyCode==RIGHT)offsetX++;
}


// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}


PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

void displayBounds() {
  noFill();
  stroke(255);
  rect(leftBound, topBound, width-rightBound-leftBound, height-bottomBound-topBound);
}


int getIndexInsideBounds() {
  return 0;
}


void mousePressed() {
  mx=mouseX;
  my=mouseY;
  loadPixels();
  //int lightLVL=-10;
  colorMode(HSB);
  color c=get(mouseX, mouseY);
  int hue= int(hue(c));
  int saturation= int(saturation(c));
  int brightness= int(brightness(c));
  //println("Integer color: "+hex(c));

 // int index=getIndexInsideBounds();
  c=color(hue(c), 255, brightness(c-50));
  //println("index: " +index);
  /*try {
    for (int i=0; i<NUMBER_PIXELS; i++) {
      arduinoPort.write(i+",0x"+hex(c).substring(2, 8)+"\0");
      println(i+",0x"+hex(c).substring(2, 8)+"\0");
    }
  }
  catch(Exception e) {
    println(e+" error");
  }*/
  println(hue, saturation, brightness);// raw
  //println(hue(c), saturation(c), brightness(c));

  /*if ((0 <=hue  && hue<15 )||( 240 <hue  && hue<=255) )  println("Red BRICK DETECTED!!!");
   else if (15 <hue  && hue<30)    println("ORANGE BRICK DETECTED!!!");
   else if (30 <hue  && hue<45)    println("YELLOW BRICK DETECTED!!!");
   else if (45 <hue  && hue<60)    println("LIMEGREEN BRICK DETECTED!!!");
   else if (60 <hue  && hue<130)    println("GREEN BRICK DETECTED!!!");
   else if (130 <hue  && hue<180)    println("BLUE BRICK DETECTED!!!");
   else if (180 <hue  && hue<200)    println("PURPLE BRICK DETECTED!!!");
   else if (200 <hue  && hue<240)    println("PINK BRICK DETECTED!!!");*/

  if (saturation> 75 && saturation<125 && brightness>25 && brightness<75) println("Black BRICK DETECTED!!!");
  if (saturation> 25 && saturation<75 && brightness>200 && brightness<250) println("Grey BRICK DETECTED!!!");
  if (saturation> 25 && saturation<25 && brightness>250 ) println("White BRICK DETECTED!!!");

  if (saturation>50 && brightness>50) println("Black BRICK DETECTED!!!");
  if (saturation>50 && brightness>50) println("Black BRICK DETECTED!!!");
  else for (int i=0; i<colorName.length; i++)if (colorThreshold[i] <=hue  && hue<colorThreshold[i+1])println(colorName[i]+" BRICK DETECTED!!!");
  // println( "x:"+mouseX, "y:"+mouseY, "z:"+kinect.getRawDepth()[mouseX + mouseY*kinect.width]);
}


void getColumn(int x,int index) {
  color row[]=new color[16]; 
  int r=0, g=0, b=0, antal=0;
  colorMode(RGB);

  loadPixels();
  for (int i=0; i<16; i++) {
    row[i] = get(x, gridSize*i+topBound+1);
    //int hue=int(hue(row[i]));
    int saturation=int(saturation(row[i]));
    int brightness=int(brightness(row[i]));

    stroke(255);
    if ((saturation> 75 && saturation<125 && brightness>25 && brightness<75 ) || (saturation> 25 && saturation<75 && brightness>200 && brightness<225) || (saturation> 25 && saturation<25 && brightness>250) ) {
      stroke(0);//ignore color range
    } else antal++;
    noFill();
    //ellipse( int(x), gridSize*i+topBound +1, 5, 5);
    //println("row" +i+" brick is color: " +row[i]);
  }
  
  for (color red : row)  r+=red(red);
  for (color green : row)   g+=green(green);
  for (color blue : row)  b+=blue(blue);
  
  average=color(r/antal, g/antal, b/antal);
  // println("color" + average);
  //  colorMode(RGB);
boolean die=false;
  //  average=color(red(average), (blue(average)<100)?0:saturation(average), brightness(average));
  if(abs(red(average)-blue(average))<25   && abs(green(average)-blue(average))<25 && abs(red(average)-green(average))<25){
    //average=color(0);
    die=true;
    println(index+" died");
  } 
  colorMode(HSB);
  
    println(index,red(average),green(average),blue(average));
    if(die)
    average=color(0,0,0);
    else average=color(hue(average), 255, brightness(average-30));

 // average=color(hue(average), (saturation(average+150)>255)?255:(saturation(average+150)), brightness(average-30));
  try {
   // if (key=='n') {
      arduinoPort.write(index+",0x"+hex(average).substring(2, 8)+"\0");
      //println(index);
      //println("amount: "+antal+" RGB "+red(average), green(average), blue(average));
   // }
  }
  catch(Exception e) {
    println(e +" antal counted:"+antal);
  }
  colorMode(RGB);
}


void displayRowCapture(int x) {
  stroke(255);
  strokeWeight(1);
  noFill();
  for (int i=0; i<16; i++) {
    ellipse( int(x), gridSize*i+topBound +1, 5, 5);
  }
}