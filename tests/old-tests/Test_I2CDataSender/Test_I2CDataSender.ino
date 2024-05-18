#include <Wire.h>
#include "I2CDataSender.h"

I2CDataSender sender = I2CDataSender();

void setup(){
  Serial.begin(115200);
  // Initialize I2C communications
  Wire.begin();  // Join I2C bus as leader
  delay(1000);

  sender.begin();
}

void loop(){
  sender.update();
}
