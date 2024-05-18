// Include Arduino Wire library for I2C
#include <Wire.h>
 
// Define follower I2C Address
#define FOLL_ADDR 0x0a

// Pins for Digital Out
#define LDR0 0
#define LDR1 1
#define LDR2 2
#define LDR3 3

// Byte to receive of I2C
byte toRec = B00000000;

void setup() {
  // Setup Serial Monitor 
  Serial.begin(115200);  
  // Initialize I2C communications as follower
  Wire.begin(FOLL_ADDR);
  Wire.onReceive(receiveEvent);

  pinMode(LDR0,


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
