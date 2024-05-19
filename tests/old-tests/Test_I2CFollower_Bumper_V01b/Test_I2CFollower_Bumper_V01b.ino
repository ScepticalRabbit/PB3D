#include <Wire.h>

// Address for nervous system slave sensor array
#define NERVSYS_ADDR 9

// Pins for collision sensors
#define COL_BUMPER_FL 0
#define COL_BUMPER_FR 1
#define COL_BUMPER_BL 8
#define COL_BUMPER_BR 9

#define COL_IR_L 2
#define COL_IR_R 7

byte collisionFlags = B00000000;

void setup() {
  Serial.begin(115200);
  Wire.begin(NERVSYS_ADDR);
  Wire.onRequest(requestEvent);
   
  pinMode(COL_BUMPER_FR, INPUT_PULLUP);
  pinMode(COL_BUMPER_FL, INPUT_PULLUP);  
  pinMode(COL_BUMPER_BR, INPUT_PULLUP);
  pinMode(COL_BUMPER_BL, INPUT_PULLUP);
  pinMode(COL_IR_R, INPUT);
  pinMode(COL_IR_L, INPUT);  
}

void requestEvent(){
  Wire.write(collisionFlags);
}

void loop() {
  byte temp = B00000000;

  int8_t switchFL = digitalRead(COL_BUMPER_FL);
  int8_t switchFR = digitalRead(COL_BUMPER_FR);
  int8_t switchBL = digitalRead(COL_BUMPER_BL);
  int8_t switchBR = digitalRead(COL_BUMPER_BR);
  int8_t IRL = digitalRead(COL_IR_L);
  int8_t IRR = digitalRead(COL_IR_R);

  // Note switches close, allow current to flow, voltage 0
  if(switchFL == 0){
    temp = temp | B00000001;
  }
  if(switchFR == 0){
    temp = temp | B00000010;
  }
  if(switchBL == 0){
    temp = temp | B00000100;
  }
  if(switchBR == 0){
    temp = temp | B00001000;
  }
  if(IRL == 0){
    temp = temp | B00010000;
  }
  if(IRR == 0){
    temp = temp | B00100000;
  }
  collisionFlags = temp;

  // Print flags to serial for debugging
  Serial.println(collisionFlags,BIN);
  
  delay(50);
}
