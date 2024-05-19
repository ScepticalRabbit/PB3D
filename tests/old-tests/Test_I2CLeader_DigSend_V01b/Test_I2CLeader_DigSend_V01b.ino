// Include Arduino Wire library for I2C
#include <Wire.h>
 
// Define Follower I2C Address
#define FOLL_ADDR 0x0a
 
byte toSend = B00000001;
 
void setup() {
  // Initialize I2C communications as Leader
  Wire.begin();

  // Start the serial
  Serial.begin(115200);
  Serial.println("I2C LEADER");
  delay(1000);
}
 
void loop() {
  
  Serial.println(toSend,BIN); 
  toSend = toSend << 1;
  if(toSend == B10000000){
    toSend = B00000001;
  }
    
  // Write a charatre to the follower
  Wire.beginTransmission(FOLL_ADDR);
  Wire.write(toSend);
  Wire.endTransmission();
  delay(100);
}
