class Particle {
  color pColor;
  int x, y;
  float size;
  boolean dead;
  Particle(int _x, int _y, float size, color _pColor)
  {
    this.size=size;
    pColor=_pColor;
    x=_x;
    y=_y;
  }
  void draw() {
    pushStyle();
    stroke(pColor);
    strokeWeight(size);  
    point(x, y);
    popStyle();
  } 
  void update() {
    if (size<1) {
      dead=true;
    } else size--;
  }
}


class SquarePulse extends Particle {

  float strokeWidth, opacity=255;

  SquarePulse(int _x, int _y, float size, color _pColor)
  {
    super(_x, _y, size, _pColor);
    this.size=size;
    strokeWidth=50;
  }
  void draw() {
    pushStyle();
  //  noFill();
    rectMode(CENTER);
    stroke(pColor, opacity);
    strokeWeight(int(strokeWidth));  
    rect(x, y, size, size);
    popStyle();
  } 
  void update() {
    if (strokeWidth<1) {
      dead=true;
    } else strokeWidth*=.9;
    size+=strokeWidth*.2;
    opacity*=.9;
  }
}