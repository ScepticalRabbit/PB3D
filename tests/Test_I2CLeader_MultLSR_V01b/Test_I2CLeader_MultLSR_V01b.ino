// Include Arduino Wire library for I2C
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
 
// Define Follower I2C Address
#define FOLL_ADDR 0x0a

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

// this holds the measurement
VL53L0X_RangingMeasurementData_t range1;
VL53L0X_RangingMeasurementData_t range2;
VL53L0X_RangingMeasurementData_t range3;

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
  readLSRs();
  delay(100);
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
  laser1.rangingTest(&range1, false); // pass in 'true' to get debug data
  laser2.rangingTest(&range2, false); 
  laser3.rangingTest(&range3, false); 

  // print sensor one reading
  Serial.print(F("L: "));
  if(range1.RangeStatus != 4){
    Serial.print(range1.RangeMilliMeter);
  } 
  else{
    Serial.print(F("Out of range"));
  }
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("R: "));
  if(range2.RangeStatus != 4){
    Serial.print(range2.RangeMilliMeter);
  } 
  else{
    Serial.print(F("Out of range"));
  }
  Serial.print(F(" "));

  // print sensor three reading
  Serial.print(F("B: "));
  if(range3.RangeStatus != 4){
    Serial.print(range3.RangeMilliMeter);
  } 
  else{
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
}

/*
#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 7
#define SHT_LOX2 6

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t range1;
VL53L0X_RangingMeasurementData_t range2;

// Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
// Keep sensor #1 awake by keeping XSHUT pin high
// Put all other sensors into shutdown by pulling XSHUT pins low
// Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
// Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_dual_sensors() {
  lox1.rangingTest(&range1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&range2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(range1.RangeStatus != 4) {     // if not out of range
    Serial.print(range1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(range2.RangeStatus != 4) {
    Serial.print(range2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();
 
}

void loop() {
   
  read_dual_sensors();
  delay(100);
}
 */
