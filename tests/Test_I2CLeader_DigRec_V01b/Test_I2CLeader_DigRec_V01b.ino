#include <Wire.h>

// Address for nervous system slave sensor array
#define FOLL_ADDR 9

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  Wire.requestFrom(FOLL,1);

  byte temp = B00000000;
  while(Wire.available()){
    temp = Wire.read();   
  }
  Serial.println(temp,BIN);
}
