// Include Arduino Wire library for I2C
#include <Wire.h>

// Include timer library
#include "PB3DTimer.h"

// Define Follower I2C Address
#define FOLL_ADDR 0x0a

// Timer for switch
uint16_t testTime = 1000;
Timer sendTimer = Timer();

bool purrOn = true;
byte toSend = B00000000;

void setup(){
  // Initialize I2C communications as Leader
  Wire.begin();

  // Start the serial
  Serial.begin(115200);
  Serial.println("I2C LEADER");
  delay(1000);

  sendTimer.start(0);
}

void loop(){
  if(sendTimer.finished()){
    sendTimer.start(testTime);
    if(purrOn){
      purrOn = !purrOn;
      toSend = B00000001;
    }
    else{
      purrOn = !purrOn;
      toSend = B00000000;
    }

    // Write a charatre to the follower
    Wire.beginTransmission(FOLL_ADDR);
    Wire.write(toSend);
    Wire.endTransmission();
  }
}
