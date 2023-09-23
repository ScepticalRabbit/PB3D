//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS - I2CDataSender
//-----------------------------------------------------------------------------
/*
TODO

Author: Lloyd Fletcher
Date Created: 11th Dec. 2022
Date Edited:  11th Dec. 2022
*/

#ifndef I2CDATASENDER_H
#define I2CDATASENDER_H

#include <Wire.h>
#include "Timer.h"

// Address for nervous system peripherial Xiao
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
// CLASS
class I2CDataSender{
public:
  I2CDataSender(){
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    // Initialise data packet variables:
    _currState.state.mood = 1;
    _currState.state.task = 2;
    _currState.state.collisionFlags[0] = true;
    _currState.state.collisionFlags[1] = false;
    _currState.state.collisionFlags[2] = true;
    _currState.state.collisionFlags[3] = false;
    _currState.state.wheelSpeed = 202.2;

      Serial.println(F("INITIAL DATA STRUCT"));
    printDataStruct();
  
    _I2CTimer.start(_I2CTime);
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_isEnabled){return;}

    if(_I2CTimer.finished()){
      _I2CTimer.start(_I2CTime);
      _sendTimer.start(0);
  
      Wire.beginTransmission(NERVSYS_ADDR);
      Wire.write(_currState.dataPacket,PACKET_SIZE);
      Wire.endTransmission();
  
      Serial.print(F("I2C Send Time: "));    
      Serial.print(_sendTimer.getTime());
      Serial.println(F("ms"));
      
      Serial.println(F("SENT DATA STRUCTURE:"));
      printDataStruct();
      // Change the data to be sent to check everything is sending ok
      changeData();
    }
  }

  //---------------------------------------------------------------------------
  // DOSOMETHING - called during the main during decision tree
  void doSomething(){
    if(!_isEnabled){return;}

    if(_startFlag){
      _startFlag = false;
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

  void setStateMood(uint8_t inMood){_currState.state.mood = inMood;}
  void setStateTask(uint8_t inTask){_currState.state.mood = inTask;}

  //---------------------------------------------------------------------------
  // DIAGNOSTIC FUNCTIONS
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

  void changeData(){
    _dataSwitch = !_dataSwitch;
    if(_dataSwitch){
      _currState.state.mood = 7;
      _currState.state.task = 8;
      _currState.state.collisionFlags[0] = true;
      _currState.state.collisionFlags[1] = true;
      _currState.state.collisionFlags[2] = true;
      _currState.state.collisionFlags[3] = true;
      _currState.state.wheelSpeed = 277.7;
    }
    else{
      _currState.state.mood = 1;
      _currState.state.task = 2;
      _currState.state.collisionFlags[0] = true;
      _currState.state.collisionFlags[1] = false;
      _currState.state.collisionFlags[2] = true;
      _currState.state.collisionFlags[3] = false;
      _currState.state.wheelSpeed = 202.2;
    }
  }

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES 
  bool _isEnabled = true;
  bool _startFlag = true;

  // DATA PACKET
  dataPacket_t _currState;
  bool _dataSwitch = false;

  // I2C VARIABLES
  Timer _I2CTimer = Timer();
  uint16_t _I2CTime = 100; // ms
  Timer _sendTimer = Timer();
};
#endif 
