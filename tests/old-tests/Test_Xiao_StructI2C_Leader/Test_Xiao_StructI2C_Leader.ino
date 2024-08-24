#include <Wire.h>
#include "PB3DTimer.h"

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
  byte data_packet[sizeof(stateData_t)];
};

#define PACKET_SIZE sizeof(stateData_t)
//----------------------------------------------------------------------------

// VARIABLES
bool dataSwitch = false;
dataPacket_t _curr_state;

Timer _I2C_timer = Timer();
uint16_t _I2C_time = 500; // ms
Timer _send_timer = Timer();

//----------------------------------------------------------------------------
// SETUP
void setup(){
  Serial.begin(115200);
  // Initialize I2C communications
  Wire.begin();  // Join I2C bus as leader
  delay(1000);

  // INIT CLASS:
  _curr_state.state.mood = 1;
  _curr_state.state.task = 2;
  _curr_state.state.collisionFlags[0] = true;
  _curr_state.state.collisionFlags[1] = false;
  _curr_state.state.collisionFlags[2] = true;
  _curr_state.state.collisionFlags[3] = false;
  _curr_state.state.wheelSpeed = 202.2;

  Serial.println(F("INITIAL DATA STRUCT"));
  printDataStruct();

  _I2C_timer.start(_I2C_time);
}

void loop(){
  if(_I2C_timer.finished()){
    _I2C_timer.start(_I2C_time);
    _send_timer.start(0);

    Wire.beginTransmission(NERVSYS_ADDR);
    Wire.write(_curr_state.data_packet,PACKET_SIZE);
    Wire.endTransmission();

    Serial.print(F("I2C Send Time: "));
    Serial.print(_send_timer.get_time());
    Serial.println(F("ms"));

    Serial.println(F("SENT DATA STRUCTURE:"));
    printDataStruct();
    changeData();
  }
}

//---------------------------------------------------------------------------
// DIAGNOSTIC FUNCTIONS
void printDataStruct(){
  Serial.print(F("Mood: "));
  Serial.print(_curr_state.state.mood);
  Serial.print(F("; "));

  Serial.print(F("TaskManager: "));
  Serial.print(_curr_state.state.task);
  Serial.print(F("; "));

  Serial.print(F("Col Flags: "));
  for(uint8_t ii = 0; ii < 4; ii++){
    if(_curr_state.state.collisionFlags[ii]){
      Serial.print(F("1"));
    }
    else{
      Serial.print(F("0"));
    }
  }
  Serial.print(F("; "));

  Serial.print(F("Speed: "));
  Serial.print(_curr_state.state.wheelSpeed);
  Serial.print(F("; "));
  Serial.println();
}

void changeData(){
  dataSwitch = !dataSwitch;
  if(dataSwitch){
    _curr_state.state.mood = 7;
    _curr_state.state.task = 8;
    _curr_state.state.collisionFlags[0] = true;
    _curr_state.state.collisionFlags[1] = true;
    _curr_state.state.collisionFlags[2] = true;
    _curr_state.state.collisionFlags[3] = true;
    _curr_state.state.wheelSpeed = 277.7;
  }
  else{
    _curr_state.state.mood = 1;
    _curr_state.state.task = 2;
    _curr_state.state.collisionFlags[0] = true;
    _curr_state.state.collisionFlags[1] = false;
    _curr_state.state.collisionFlags[2] = true;
    _curr_state.state.collisionFlags[3] = false;
    _curr_state.state.wheelSpeed = 202.2;
  }
}
