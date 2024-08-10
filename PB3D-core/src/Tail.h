//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TAIL_H
#define TAIL_H

#include <Arduino.h>
#include <Wire.h> // I2C
//#include "CollisionManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Timer.h"
#include "Speaker.h"

#include <Servo.h>

#define TAIL_SERVO_POUT 8

#define TAIL_CENT 0
#define TAIL_SET_POS 1
#define TAIL_WAG_CON 2
#define TAIL_WAG_INT 3

class Tail{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  Tail(){
  }

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin(){
    // Servo - for wagging tail
    _tailServo.attach(TAIL_SERVO_POUT);
    // Timer start
    _wagTimer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update(){
    if(!_isEnabled){return;}

    // Update the tail servo based on the main timer
    if(_updateTimer.finished()){
      _updateTimer.start(_updateInt);

      // Decision tree based on tail state - set by tasks+mood
      if(_currState == TAIL_SET_POS){
        _tailServo.write(_tailPosCurr);
      }
      else if(_currState == TAIL_WAG_CON){
        wagContinuous();
      }
      else if(_currState == TAIL_WAG_INT){
        wagInterval();
      }
      else{
        _tailServo.write(_tailPosCent);
      }
    }
  }

  //---------------------------------------------------------------------------
  // TAIL FUNCTIONS
  void wagContinuous(){
    if(_wagTimer.finished()){
      _wagTimer.start(_wagMoveTime);
      if(_wagSwitch){
        _wagSwitch = !_wagSwitch;
        _tailServo.write(_tailPosCent-_wagPosOffset);
      }
      else{
        _wagSwitch = !_wagSwitch;
        _tailServo.write(_tailPosCent+_wagPosOffset);
      }
    }
  }

  void wagInterval(){
    if(_wagTimer.finished()){
      if(_wagCount<_wagCountLim){
        _wagTimer.start(_wagMoveTime);

        if(_wagSwitch){
          _wagSwitch = !_wagSwitch;
          _tailServo.write(_tailPosCent-_wagPosOffset);
        }
        else{
          _wagSwitch = !_wagSwitch;
          _tailServo.write(_tailPosCent+_wagPosOffset);
          _wagCount++;
        }
      }
      else{
        _wagTimer.start(_wagPauseTime);
        _tailServo.write(_tailPosCent);
        _wagCount = 0;
      }
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

  void setState(uint8_t inState){_currState = inState;}
  void setPos(int16_t inPos){_tailPosCurr = inPos;}

  void setWagMoveTime(uint16_t inMoveTime){_wagMoveTime = inMoveTime;}
  void setWagPosOffset(int16_t inOffset){_wagPosOffset = inOffset;}
  void setWagPauseTime(uint16_t inPauseTime){_wagPauseTime = inPauseTime;}
  void setWagCountLim(uint8_t inCountLim){_wagCountLim = inCountLim;}

  void setWagParams(uint16_t inMoveTime, int16_t inOffset, uint16_t inPauseTime, uint8_t inCountLim){
    _wagMoveTime = inMoveTime;
    _wagPosOffset = inOffset;
    _wagPauseTime = inPauseTime;
    _wagCountLim = inCountLim;
  }

  void setWagConParams(uint16_t inMoveTime, int16_t inOffset){
    _wagMoveTime = inMoveTime;
    _wagPosOffset = inOffset;
  }

  void setWagIntParams(uint16_t inMoveTime, int16_t inOffset, uint16_t inPauseTime, uint8_t inCountLim){
    _wagMoveTime = inMoveTime;
    _wagPosOffset = inOffset;
    _wagPauseTime = inPauseTime;
    _wagCountLim = inCountLim;
  }

  void reset(){
    _currState = TAIL_CENT;
    _wagCount = 0;
  }

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _isEnabled = true;

  Servo _tailServo;

  uint8_t _currState = TAIL_CENT;
  uint16_t _updateInt = 50;
  Timer _updateTimer = Timer();

  // MODE: SET POS
  int16_t _tailPosCurr = 90;
  int16_t _tailPosCent = 90;

  // MODE: WAG CONTINUOUS
  bool _wagSwitch = true;
  int16_t _wagPosOffset = 30;
  uint16_t _wagMoveTime = 400;
  Timer _wagTimer = Timer();

  // MODE: WAG INTERVAL
  uint8_t _wagCountLim = 4;
  uint8_t _wagCount = 0;
  uint16_t _wagPauseTime = 4000;
};
#endif // END CLASS TAIL
