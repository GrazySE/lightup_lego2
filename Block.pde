





class Block {
  final int blockInterval=15;
  PVector coord = new PVector();
  int size; 
  boolean dead;
  color fillColor=color(255);
  Block(int x, int y, int size, color _fillColor) {
    coord.set(x, y);
    this.size=size;
    fillColor=_fillColor;
  }


  void display() {
    pushStyle();
    rectMode(CENTER);
    fill(fillColor);
    stroke(fillColor, 100);
    rect(coord.x, coord.y, size, size);
    popStyle();
  }

  void update() {
  }
  void reset() {
  }
  
  void kill(){
  dead=true;
  }
}


class MusicBlock extends Block {
  boolean  highLight=true, triggered;
  float amp=0;
  Oscil   wave;
  Wavetable form;
  MusicBlock(int x, int y, int size, color _fillColor) {
    super( x, y, size, _fillColor);
    // create a sine wave Oscil, set to 440 Hz, at 0.5 amplitude
    //   final String colorName[]={"RED", "ORANGE", "YELLOW", "LIMEGREEN", "GREEN", "BLUE", "PURPLE", "PINK", "RED"};
    // final int colorThreshold[]={0, 15, 30, 45, 60, 130, 180, 200, 240, 255};
    colorMode(HSB);
    int hue=int(hue(_fillColor));

    if ( colorThreshold[0]<hue && colorThreshold[1]>hue) {
      form=Waves.SINE;
    } else  if ( colorThreshold[1]<hue && colorThreshold[2]>hue) {
      form=Waves.SAW;
    } else if ( colorThreshold[2]<hue && colorThreshold[3]>hue) {
      form=Waves.PHASOR;
    } else if ( colorThreshold[3]<hue && colorThreshold[4]>hue) {
      form=Waves.QUARTERPULSE;
    } else if ( colorThreshold[4]<hue && colorThreshold[5]>hue) {
      form=Waves.SQUARE;
    } else if ( colorThreshold[5]<hue && colorThreshold[6]>hue) {
      form=Waves.SQUARE;
    } else if ( colorThreshold[6]<hue && colorThreshold[7]>hue) {
      form=Waves.TRIANGLE;
    } else if ( colorThreshold[7]<hue && colorThreshold[8]>hue) {
      form=Waves.TRIANGLE;
    } else  if ( colorThreshold[8]<hue && colorThreshold[9]>hue) {
      form=Waves.SINE;
    } else    form=Waves.SINE;

    wave = new Oscil( height-y, amp, form );


    // patch the Oscil to the output
    wave.patch( out );
   // particleList.add(new SquarePulse(int(coord.x), int(coord.y), size, _fillColor));
       if (musicLine>coord.x)triggered=true;


  }

  void display() {
    pushStyle();
    rectMode(CENTER);
    fill(fillColor);
    rect(coord.x, coord.y, size, size);
    popStyle();
  }

  void update() {
    super.update();
    if (musicLine>coord.x&&!triggered)trigger();
    wave.setAmplitude( amp );
    if (amp>0)amp*=0.9;
    else amp=0;
  }

  void trigger() {

    triggered=true;
    particleList.add(new SquarePulse(int(coord.x), int(coord.y), 50, color(fillColor)));
    musicLineWidth=15;
    amp=0.6;
  }
  void reset() {
    triggered=false;
  }
  void kill(){
  dead=true;
    wave.unpatch(out);
  }
}