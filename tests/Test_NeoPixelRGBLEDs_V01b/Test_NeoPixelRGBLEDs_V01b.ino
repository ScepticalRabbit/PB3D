#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN 6
 
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 4

bool flag = true;
 
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin();
  pixels.show();
  delay(500);
}

void loop() {

  if(flag){
    pixels.setPixelColor(0,0,0,255);
    pixels.setPixelColor(1,0,0,255);
    pixels.setPixelColor(2,0,0,255);
    pixels.setPixelColor(3,0,0,255);
    pixels.show();
    flag = !flag;
  }
  else{
    pixels.setPixelColor(0,0,255,0);
    pixels.setPixelColor(1,0,255,0);
    pixels.setPixelColor(2,0,255,0);
    pixels.setPixelColor(3,0,255,0);
    pixels.show();
    flag = !flag;
  }
  delay(1000);
}
