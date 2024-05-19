#include <Wire.h>
#include "Timer.h"

// Address for nervous system peripherial sensor array
#define NERVSYS_ADDR 9

//----------------------------------------------------------------------------
// DATA STRUCTURES
// Declare data structure and union types
typedef struct stateData_t{
  uint8_t mood;
  uint8_t task;
  bool collisionFlags[4];
  float wheelSpeed;  
};

typedef union dataPacket_t{
  stateData_t state;
  byte dataPacket[sizeof(stateData_t)];
};

#define PACKET_SIZE sizeof(stateData_t)
//----------------------------------------------------------------------------

// VARIABLES
dataPacket_t _currState;

Timer _printTimer = Timer();
uint16_t _printTime = 500; // ms

//----------------------------------------------------------------------------
// I2C HANDLER FUNCTIONS
void requestEvent(){
  
}

void receiveEvent(int bytesRec){
  int16_t ii = 0;
  while(0<Wire.available()){
    if(ii < PACKET_SIZE){
      _currState.dataPacket[ii] = Wire.read();
    }
    else{
      Wire.read();
    }
    ii++;
  }
}

//----------------------------------------------------------------------------
// SETUP
void setup() {
  Serial.begin(115200);
  Wire.begin(NERVSYS_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  // INIT CLASS:
  _currState.state.mood = 0;
  _currState.state.task = 0;
  _currState.state.collisionFlags[0] = false;
  _currState.state.collisionFlags[1] = false;
  _currState.state.collisionFlags[2] = false;
  _currState.state.collisionFlags[3] = false;
  _currState.state.wheelSpeed = 0.0;
  
  Serial.println(F("INITIAL DATA STRUCT"));
  printDataStruct();

  _printTimer.start(_printTime);
}

//----------------------------------------------------------------------------
// MAIN LOOP
void loop() {
  if(_printTimer.finished()){
    _printTimer.start(_printTime);
    
    Serial.println(F("REC DATA STRUCTURE:"));
    printDataStruct(); 
  }
}

// FUNCTIONS
void printDataStruct(){
  Serial.print(F("Mood: "));
  Serial.print(_currState.state.mood);
  Serial.print(F("; "));
  
  Serial.print(F("TaskManager: "));
  Serial.print(_currState.state.task);
  Serial.print(F("; "));

  Serial.print(F("Col Flags: "));
  for(uint8_t ii = 0; ii < 4; ii++){
    if(_currState.state.collisionFlags[ii]){
      Serial.print(F("1"));
    }
    else{
      Serial.print(F("0"));
    }
  }
  Serial.print(F("; "));

  Serial.print(F("Speed: "));
  Serial.print(_currState.state.wheelSpeed);
  Serial.print(F("; "));
  Serial.println();
}
