

// Daniel Shiffman
// All features test

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

/**------------------------------------------------------------//
 //                                                            //
 //  IDK  - Prototype LIGHTUP LEGO2                            //
 //  av: Alrik He    v.0.2.1                                   //
 //  Malmö högskola                                            //
 //                                                            //
 //      2016-12-18    -     2017-01-11                        //
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
enum GAMEMODE {
  MERGE, MUSIC
};
GAMEMODE gameMode=GAMEMODE.MUSIC;
final int[]  black={100, 50}, grey={50, 200}, white={25, 252};
int mx, my, interval=100, threshold=2, colorDiff=18, hueOffset=-22, modifierR=1, modifierG=5, modifierB=-1;
float deg;
String portName="";
boolean ir = false;
boolean colorDepth = false;
boolean mirror = false;
byte gridSize=12;
int rows=20, columns=20, offsetX, offsetY;
boolean toggle, shift, grid;
int [][] averageColor = new int[200][150];
color average = color(255);
int offset=int(gridSize*0.25), senseWidth, senseHeight;
final int leftBound=210, topBound=105, rightBound=250, bottomBound=190;
final String colorName[]={"RED", "ORANGE", "YELLOW", "LIMEGREEN", "GREEN", "BLUE", "PURPLE", "PINK", "RED"};
final int colorThreshold[]={0, 15, 30, 45, 60, 130, 180, 200, 240, 255};
final byte NUM_OF_RGB_LED=40;
long timer=6000, musicTimer=6000;
int musicInterval, musicLine, musicLineWidth;
// Angle for rotation
float a = 0;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];
int min=40, max=1000, pixelSize=gridSize;
final int NUMBER_PIXELS=40; // strip RGBLEDs
//final int BAUDRATE= 19200;
final int BAUDRATE= 38400;

// GAMES


ArrayList<Block> blockList= new ArrayList<Block>(); 
ArrayList<Particle> particleList= new ArrayList<Particle>(); 

import ddf.minim.*;
import ddf.minim.ugens.*;

Minim       minim;
AudioOutput out;


void setup() {

  minim = new Minim(this);
  // use the getLineOut method of the Minim object to get an AudioOutput object
  out = minim.getLineOut();

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
  //size(640, 480, P3D);
  size(2000, 800, P3D);

  // fullScreen();
  // kinect = new Kinect(this);
  //kinect.initDepth();
  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  try {
    printArray(Serial.list());
    //portName= Serial.list()[0];
    // mac: portName ="/dev/cu.usbmodemFD121";
    portName= "COM98";
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

  /* senseWidth=width-leftBound-rightBound;
   senseHeight=height-topBound-bottomBound;*/

  senseWidth=kinect.width-leftBound-rightBound;
  senseHeight=kinect.height-topBound-bottomBound;
}


void draw() {
  background(0);
  if (shift) {
    colorMode(HSB);
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

    pushMatrix();
    translate(kinect.width*.5, kinect.height*.5);
    scale(1.15);


    image(kinect.getVideoImage(), -kinect.width*.5, -kinect.height*.5);
    // Translate and rotate

    // image(kinect.getDepthImage(), 640, 0);
    fill(255);

    popMatrix();
    if (toggle) {
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


      // if (toggle) {
      strokeWeight(1);
      for (int i=0; i<width; i+=gridSize)for (int j=0; j<height; j+=gridSize) {
        fill(averageColor[int(i/gridSize)][int(j/gridSize)]); 
        //point(i+gridSize*.5, j+gridSize*.5);
        rect(i, j, gridSize, gridSize);
      }
    }

    /*text(
     "Press 'i' to enable/disable between video image and IR image,  " +
     "Press 'c' to enable/disable between color depth and gray scale depth,  " +
     "Press 'm' to enable/diable mirror mode, "+
     "UP and DOWN to tilt camera   " +
     "Framerate: " + int(frameRate), 10, 515);*/
  }

  //displayRowCapture(mx);
  switch(gameMode) {
  case MERGE:
    if (timer+interval<millis()) {
      timer=millis();
      // for(int i=0; i<16;i++) getColumn(mx+i*gridSize,int(i*2.5));
      //  for(int i=0; i<20;i++) getColumn(mx+i*gridSize,int(i*2));
      for (int i=0; i<40; i++) getColumn(mx+i*5, i);
    }
    if (grid) displayGrid();

    /*  for (Block b : blockList) {
     b.update();
     b.display();
     }*/

    break;
  case MUSIC:
    ellipse(mouseX, mouseY, 10, 10);
    if (timer+interval<millis()) {
      timer=millis();
      for (int i=leftBound; i<kinect.width-rightBound; i+=gridSize*.5) {
        for (int j=topBound; j<kinect.height-bottomBound; j+=gridSize*.5) {
          color temp=getCell(i+1, j+2);
          int x  = int(map(i, leftBound, kinect.width-rightBound, 0, width));
          int y   =int(map(j, topBound, kinect.height-bottomBound, 0, height));
          boolean create=true;
          for (Block b : blockList) {
            if (b.coord.x==x&& b.coord.y==y) {
              create=false;
            }
          }
          if (temp==color(0)) {
            create=false;
            for (Block b : blockList) {
              if (b.coord.x==x&& b.coord.y==y) {
                b.kill();
              }
            }
          }

          if (create) {
            blockList.add(new  MusicBlock(x, y, 50, temp));
          }
        }
      }
      int index=int(map(musicLine, 0, width, 40, 0));

      writeColorToindex(color(255), index);
      //reset
      if (index==40)writeColorToindex(color(0), 0);
      else writeColorToindex(color(0), index+1);
    }
    displayTempoBars();

    noFill();
    stroke(255);
    strokeWeight(musicLineWidth);

    line(musicLine, 0, musicLine, height);

    if (musicLine>width) {
      musicLine=0;
      for (Block b : blockList) {
        b.reset();
      }
    } else 
    musicLine+=5;
    if (musicLineWidth<=5)musicLineWidth=5;
    else musicLineWidth--;
    // musicLine= int((millis()*0.5)%width);

    for (Block b : blockList) {
      b.update();
      b.display();
      if (b.dead) {
        blockList.remove(b);
        break;
      }
    }
    for (Particle p : particleList) {
      p.update();
      p.draw();
      if (p.dead) {
        particleList.remove(p);
        break;
      }
    }

    break;
  }
  displayBounds();
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
  if (key=='g')grid=!grid;
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
  pushStyle();
  noFill();
  strokeWeight(3);
  stroke(random(255));
  //rect(leftBound, topBound, width-rightBound-leftBound, height-bottomBound-topBound);
  rect(leftBound, topBound, kinect.width-rightBound-leftBound, kinect.height-bottomBound-topBound);
  strokeWeight(3);
  popStyle();
}

int getIndexInsideBounds() {
  return 0;
}


void mousePressed() {
  /*  mx=mouseX;
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
   try {
   for (int i=0; i<NUMBER_PIXELS; i++) {
   arduinoPort.write(i+",0x"+hex(c).substring(2, 8)+"\0");
   println(i+",0x"+hex(c).substring(2, 8)+"\0");
   }
   }
   catch(Exception e) {
   println(e+" error");
   }
   println(hue, saturation, brightness);// raw
   
   
   if (saturation> 75 && saturation<125 && brightness>25 && brightness<75) println("BLACK BRICK DETECTED!!!");
   if (saturation> 25 && saturation<75 && brightness>200 && brightness<250) println("GREY BRICK DETECTED!!!");
   if (saturation> 25 && saturation<25 && brightness>250 ) println("WHITE BRICK DETECTED!!!");
   
   if (saturation>50 && brightness>50) println("BLACK BRICK DETECTED!!!");
   if (saturation>50 && brightness>50) println("BLACK BRICK DETECTED!!!");
   else for (int i=0; i<colorName.length; i++)if (colorThreshold[i] <=hue  && hue<colorThreshold[i+1])println(colorName[i]+" BRICK DETECTED!!!");
   // println( "x:"+mouseX, "y:"+mouseY, "z:"+kinect.getRawDepth()[mouseX + mouseY*kinect.width]);
   */
  colorMode(HSB);
  blockList.add(new  MusicBlock(mouseX, mouseY, 50, color(random(255), 255, 200)));
  particleList.add(new SquarePulse(mouseX, mouseY, 50, color(255)));
}


void getColumn(int x, int index) {
  color row[]=new color[16]; 
  int r=0, g=0, b=0, antal=0;
  colorMode(RGB);
  loadPixels();
  for (int i=0; i<16; i++) {
    row[i] = get(x, gridSize*i+topBound+1);
    //int hue=int(hue(row[i]));
    int saturation=int(saturation(row[i]));
    int brightness=int(brightness(row[i]));

    //stroke(255);
    if ((saturation> 100-threshold && saturation<100+threshold && brightness>50-threshold && brightness<50+threshold ) || 
      (saturation> 50-threshold && saturation<50+threshold && brightness>200-threshold && brightness<200+threshold) ||
      (saturation> 25-threshold && saturation<25+threshold && brightness>252-threshold) ) {
      //stroke(0);//ignore color range
    } else antal++;
    //noFill();
    //ellipse( int(x), gridSize*i+topBound +1, 5, 5);
    //println("row" +i+" brick is color: " +row[i]);
  }

  /*for (color red : row)  r+=red(red)+4;
   for (color green : row)   g+=green(green);
   for (color blue : row)  b+=blue(blue)-2;*/
  for (color red : row)  r+=red(red)+modifierR;
  for (color green : row)   g+=green(green)+modifierG;
  for (color blue : row)  b+=blue(blue)+modifierB;
  if (antal==0) {
    average=color(0);
  } else
    average=color(r/antal, g/antal, b/antal);
  // println("color" + average);
  //  colorMode(RGB);
  boolean die=false;
  //  average=color(red(average), (blue(average)<100)?0:saturation(average), brightness(average));
  if (abs(red(average)-blue(average))<colorDiff   && abs(green(average)-blue(average))<colorDiff && abs(red(average)-green(average))<colorDiff) {
    //average=color(0);
    die=true;
    println(index+" died");
  } 
  colorMode(HSB);
  //println(index,red(average),green(average),blue(average));
  if (die)
    average=color(0, 0, 0);
  else average=color(hue(average)+hueOffset, 255, brightness(average-5));

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
color getCell(int x, int y) {
  color cell;
  int r=0, g=0, b=0;
  boolean die=false;

  //colorMode(RGB);
  //loadPixels();
  cell = get(x, y);
  int saturation=int(saturation(cell));
  int brightness=int(brightness(cell));

  if ((saturation> black[0]-threshold && saturation<black[0]+threshold && brightness>black[1]-threshold && brightness<black[1]+threshold ) || 
    (saturation> grey[0]-threshold && saturation<grey[0]+threshold && brightness>grey[1]-threshold && brightness<grey[1]+threshold) ||
    (saturation> white[0]-threshold && saturation<white[0]+threshold && brightness>white[1]-threshold) ) {
    die=true;
  } 

  r=int(red(cell)+modifierR);
  g=int(green(cell)+modifierG);
  b=int(blue(cell)+modifierB);

  average=color(r, g, b);

  if (abs(red(average)-blue(average))<colorDiff   && abs(green(average)-blue(average))<colorDiff && abs(red(average)-green(average))<colorDiff) {
    die=true;
  } 

  colorMode(HSB);
  if (die)  average=color(0, 0, 0);
  else average=color(hue(average)+hueOffset, 255, brightness(average-5));
  //stroke(255);
  // point(x, y);

  return average;
}

void writeColorToindex(color c, int i) {
  try {
    arduinoPort.write(i+",0x"+hex(c).substring(2, 8)+"\0");
  }
  catch(Exception e) {
    println("cant write to index "+i);
  }
}

void displayRowCapture(int x) {
  stroke(255);
  strokeWeight(1);
  noFill();
  for (int i=0; i<16; i++) {
    ellipse( int(x), gridSize*i+topBound +1, 5, 5);
  }
}
void displayGrid() {
  strokeWeight(1);
  stroke(0, 0, 0);
  for (int i=0; i<kinect.width; i+=gridSize)line(i+offsetX, 0+offsetY, i+offsetX, kinect.height+offsetY);
  for (int i=0; i<kinect.height; i+=gridSize)line(0+offsetX, i+offsetY, kinect.width+offsetX, i+offsetY);
}
void displayTempoBars() {



  strokeWeight(1);
  for (int i=0; i<width; i++) {
    if (i%4==1) { 
      stroke(100);
      strokeWeight(3);
    } else {
      stroke(50);
      strokeWeight(1);
    }
    line(i*width/32, 0, i*width/32, height);
  }
}