// Include Arduino Wire library for I2C
#include <Wire.h>
 
// Define follower I2C Address
#define FOLL_ADDR 0x0a
 
byte toRec = B00000000;

void setup() {
  // Initialize I2C communications as follower
  Wire.begin(FOLL_ADDR);
   
  // Function to run when data received from master
  Wire.onReceive(receiveEvent);
  
  // Setup Serial Monitor 
  Serial.begin(115200);
  Serial.println("I2C FOLLOWER");
  delay(1000);
}

void receiveEvent(int numBytes) {
  // Read one byte from the I2C
  toRec = Wire.read();
}
 
void loop() {
  // Print value of incoming byte
  Serial.println(toRec,BIN); 
  delay(100);
}
