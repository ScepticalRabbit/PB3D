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
#include "PB3DTimer.h"

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
  byte data_packet[sizeof(stateData_t)];
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

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(!_enabled){return;}

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
      // Change the data to be sent to check everything is sending ok
      changeData();
    }
  }

  //---------------------------------------------------------------------------
  // DOSOMETHING - called during the main during decision tree
  void doSomething(){
    if(!_enabled){return;}

    if(_start_flag){
      _start_flag = false;
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

  void setStateMood(uint8_t inMood){_curr_state.state.mood = inMood;}
  void setStateTask(uint8_t inTask){_curr_state.state.mood = inTask;}

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
    _data_switch = !_data_switch;
    if(_data_switch){
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

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;
  bool _start_flag = true;

  // DATA PACKET
  dataPacket_t _curr_state;
  bool _data_switch = false;

  // I2C VARIABLES
  Timer _I2C_timer = Timer();
  uint16_t _I2C_time = 100; // ms
  Timer _send_timer = Timer();
};
#endif
