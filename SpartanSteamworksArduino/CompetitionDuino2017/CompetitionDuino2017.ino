#include <Wire.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define R_STRIP_PIN 6
#define L_STRIP_PIN 5

//Define colors
#define RED  strip.Color(255,0,0)
#define YELLOW  strip.Color(255,255,0)
#define GREEN  strip.Color(0,255,0)
#define BLUE  strip.Color(0,0,255)
#define WHITE  strip.Color(255,255,255)
// DEFINE MODES
#define MODE_YELLOW 0
#define MODE_PULSATINGYELLOW 1

#define MODE_RED 2
#define MODE_PULSATINGYELLOWRED 4
#define MODE_PULSATINGRED 3

#define MODE_BLUE 5
#define MODE_PULSATINGBLUE 6
#define MODE_PULSATINGYELLOWBLUE 7

#define MODE_GROOVY 8

int Mode = MODE_PULSATINGYELLOW;
int currentMode = -1;
int strip_mode = 0;

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(18, R_STRIP_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(18, L_STRIP_PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() { 
  Serial.begin(9600);
  
  Wire.begin(8);
  Wire.onReceive(receiveEvent);

  strip.begin();
  strip.show();

  strip2.begin();
  strip.show();
  }

void loop() {
  while(Serial.available()) {
    char inChar = Serial.read();
    String inString = "";
    inString += inChar;

    Mode = inString.toInt();
  }

  if(currentMode != Mode) {
    switch(Mode) {
      case MODE_YELLOW:
        Serial.write("switchingMode: Yellow");
        changeStripColor(YELLOW,18,strip);
        changeStripColor(YELLOW,18,strip2);
        break;
      case MODE_PULSATINGYELLOW:

        
        Serial.write("switchingMode: PulsatingYellow");
        pulse(255, 255, 0, 0, 0, 0, 100);
        break;
        
      case MODE_RED:
      
        Serial.write("switchingMode: RED");
        changeStripColor(RED,18,strip);
        changeStripColor(RED,18,strip2);
        break;
      case MODE_PULSATINGRED:
      
        Serial.write("switchingMode: PulsatingRED");
        pulse(255, 0, 0, 0, 0, 0, 100);
        break;
      case MODE_PULSATINGYELLOWRED:
      
        Serial.write("switchingMode: PulsatingYellowRED");
        pulse(255, 255, 0, 255, 0, 0, 100);
        break;
        
      case MODE_BLUE:
        Serial.write("switchingMode: BLUE");
        changeStripColor(RED,18,strip);
        changeStripColor(RED,18,strip2);
        break;
      case MODE_PULSATINGBLUE:
      
        Serial.write("switchingMode: PulsatingBLUE");
        pulse(0, 0, 255, 0, 0, 0, 100);
        break;
      case MODE_PULSATINGYELLOWBLUE:
      
        Serial.write("switchingMode: PulsatingYellowBLUE");
        pulse(255, 255, 0, 0, 0, 255, 100);
        break;
      case MODE_GROOVY:
        
        Serial.write("switchingMode: GROOVY");
        changeStripColor(GREEN,18,strip);
        changeStripColor(GREEN,18,strip2);
        break;
    }
    currentMode = Mode;
  }
  pulseUpdate();

  delay(50);
}

void receiveEvent(int howMany) {
  while(Wire.available() > 0) {
    byte inByte = Wire.read();
    Mode = inByte;
  }
}
//CHANGES THE ENTIRE STRIP TO A SINGULAR COLOR
void changeStripColor(uint32_t c, int striplength, Adafruit_NeoPixel currentStrip){
  for(int i = 0; i <striplength;i++){
   currentStrip.setPixelColor(i, c);
  }
}
//OldStolenCode - to be redone


// A pulsating color
uint8_t pulse_r;
uint8_t pulse_g;
uint8_t pulse_b;
uint8_t pulse_r2;
uint8_t pulse_g2;
uint8_t pulse_b2;
double pulse_s;
double pulse_amount;
void pulse(uint8_t r, uint8_t g, uint8_t b, uint8_t r2, uint8_t g2, uint8_t b2, long duration) {
  pulse_r = r;
  pulse_g = g;
  pulse_b = b;
  pulse_r2 = r2;
  pulse_g2 = g2;
  pulse_b2 = b2;
  pulse_s = 0;
  pulse_amount = 0.1;

  strip_mode = 2;
}
void pulseUpdate() {
  pulse_s += pulse_amount;
  
  if(strip_mode == 2) {
    uint8_t r_amplitude = (pulse_r - pulse_r2) / 2;
    uint8_t g_amplitude = (pulse_g - pulse_g2) / 2;
    uint8_t b_amplitude = (pulse_b - pulse_b2) / 2;

    uint8_t r = r_amplitude * (sin(pulse_s) / 2 + 0.5) + pulse_r2;
    uint8_t g = g_amplitude * (sin(pulse_s) / 2 + 0.5) + pulse_g2;
    uint8_t b = b_amplitude * (sin(pulse_s) / 2 + 0.5) + pulse_b2;
    
    for(int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i,
        r,
        g,
        b);
    }
    strip.show();
    for(int i = 0; i < strip2.numPixels(); i++) {
      strip2.setPixelColor(i,
        r,
        g,
        b);
    }
    strip2.show();
  }
//  for(int i = 0; i < strip.numPixels(); i++) {
//    strip.setPixelColor(pulse_i, pulse_r * (sin(millis() / pulse_duration) + 1 / 2) * 200, pulse_g * (sin(millis() / pulse_duration) + 1 / 2) * 200, pulse_b * (sin(millis() / pulse_duration) + 1 / 2) * 200);
//  }
//  strip.show();
}

