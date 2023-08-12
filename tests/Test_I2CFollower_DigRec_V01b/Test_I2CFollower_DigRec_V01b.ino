#include <Wire.h>

// Address for nervous system slave sensor array
#define FOLL_ADDR 9

byte byteToSend = B00000111;

void setup() {
  Serial.begin(115200);
  Wire.begin(FOLL_ADDR);
  Wire.onRequest(requestEvent);
   
}

void requestEvent(){
  Wire.write(byteToSend);
}

void loop() {
  /*
  byteToSend = byteToSend << 1;
  if(byteToSend == B10000000){
    byteToSend = B00000001;
  }
  */
  byteToSend = B00000111;

  // Print flags to serial for debugging
  Serial.println(byteToSend,BIN);
  
  delay(50);
}
