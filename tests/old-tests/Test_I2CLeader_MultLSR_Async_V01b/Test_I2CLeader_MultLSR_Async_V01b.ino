// Include Arduino Wire library for I2C
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "PB3DTimer.h"

// Define Follower I2C Address
#define FOLL_ADDR 0x11

// Define addresses for all laser sensors
#define ADDR_LSR1 0x30
#define ADDR_LSR2 0x31
#define ADDR_LSR3 0x32

// Laser Sensor Variables
uint16_t resetDelay = 50;

// objects for the vl53l0x
Adafruit_VL53L0X laser1 = Adafruit_VL53L0X();
Adafruit_VL53L0X laser2 = Adafruit_VL53L0X();
Adafruit_VL53L0X laser3 = Adafruit_VL53L0X();

bool rangeFlag1 = false, rangeFlag2 = false, rangeFlag3 = false;
uint16_t range1 = 0, range2 = 0, range3 = 0;
uint16_t rangeInt = 500;
Timer rangeTimer = Timer();

// I2C send byte
byte toSend = B00000000;

void setup() {
  // Initialize I2C communications as Leader
  Wire.begin();
  // Start the serial
  Serial.begin(115200);

  // Need to delay to make sure the I2C follower has started
  delay(2000);
  setLSRAddrs();
}

void loop() {
  uint32_t startTime = millis();
  readLSRs();
  uint32_t endTime = millis();

  /*
  Serial.print(F("LOOP TOOK: "));
  Serial.print(endTime-startTime);
  Serial.println();
  */
}

void sendByteWithI2C(byte inByte){
  Wire.beginTransmission(FOLL_ADDR);
  Wire.write(inByte);
  Wire.endTransmission();
}

void setLSRAddrs(){
  // Reset all laser sensors - set all low
  toSend = B00000000;
  sendByteWithI2C(toSend);
  delay(resetDelay);

  // Turn on all sensors - set all high
  toSend = B00001110;
  sendByteWithI2C(toSend);
  delay(resetDelay);

  // Activate first laser sensor
  toSend = B00000010;
  sendByteWithI2C(toSend);
  delay(resetDelay);
  // Initiate laser 1
  if(!laser1.begin(ADDR_LSR1)){
    Serial.println(F("Failed to boot first laser"));
    while(1);
  }
  else{
    Serial.println(F("First laser initialised"));
  }
  delay(resetDelay);

  // Activate second laser sensor
  toSend = B00000110;
  sendByteWithI2C(toSend);
  delay(resetDelay);
  // Initiate laser 2
  if(!laser2.begin(ADDR_LSR2)) {
    Serial.println(F("Failed to boot second laser"));
    while(1);
  }
  else{
    Serial.println(F("Second laser initialised"));
  }
  delay(resetDelay);

  // Activate second laser sensor
  toSend = B00001110;
  sendByteWithI2C(toSend);
  delay(resetDelay);
  // Initiate laser 3
  if(!laser3.begin(ADDR_LSR3)) {
    Serial.println(F("Failed to boot third laser"));
    while(1);
  }
  else{
    Serial.println(F("Third laser initialised"));
  }
  delay(resetDelay);
}

void readLSRs() {
  if(rangeTimer.finished()){
    rangeTimer.start(rangeInt);

    Serial.print(F("L: "));
    Serial.print(range1);
    Serial.print(F(" "));
    Serial.print(F("R: "));
    Serial.print(range2);
    Serial.print(F(" "));
    Serial.print(F("B: "));
    Serial.print(range3);
    Serial.println();

    laser1.startRange();
    laser2.startRange();
    laser3.startRange();
    rangeFlag1 = false;
    rangeFlag2 = false;
    rangeFlag3 = false;
  }

  if(laser1.isRangeComplete() && !rangeFlag1){
    range1 = laser1.readRangeResult();
    rangeFlag1 = true;
  }
  if(laser2.isRangeComplete() && !rangeFlag2){
    range2 = laser2.readRangeResult();
    rangeFlag2 = true;
  }
  if(laser3.isRangeComplete() && !rangeFlag3){
    range3 = laser3.readRangeResult();
    rangeFlag3 = true;
  }
}
