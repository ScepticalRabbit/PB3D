#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskPickedUp.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKPICKEDUP_H
#define TASKPICKEDUP_H

#include <Wire.h> // I2C
#include <Seeed_CY8C401XX.h> // Capcitive Touch Sensor
#include "MoodManager.h"
#include "CollisionManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Timer.h"
#include "Speaker.h"
#include "PatSensor.h"

// Define follower Xiao I2C Address
#ifndef ADDR_FOLLBOARD
  #define ADDR_FOLLBOARD 0x11
#endif

class TaskPickedUp{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskPickedUp(CollisionManager* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
               Speaker* inSpeaker, PatSensor* inPatSens);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // PICKED UP - called during the main during decision tree
  void pickedUp();

  //---------------------------------------------------------------------------
  // Get, set and reset
  bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  // MAIN OBJECT POINTERS
  CollisionManager* _collisionObj = NULL;
  MoodManager* _moodObj = NULL;
  TaskManager* _taskObj = NULL;
  MoveManager* _moveObj = NULL;
  Speaker* _speakerObj = NULL;
  PatSensor* _patSensObj = NULL;

  // TASK - PICKED UP
  bool _isEnabled = true;
  bool _isPickedUp = false;
  bool _startPickedUpFlag = false;
  bool _patFlag = false, _patComplete = false;

  bool _exitFlag = false, _exitTimerOn = false;
  uint16_t _exitTime = 2000;
  Timer _exitTimer = Timer();

  // Call timer and update time
  uint16_t _callUpdateTime = 2000;
  Timer _callTimer = Timer();

  // Variables for purring
  byte _sendByte = B00001110;
  bool _purrOnFlag = false;
  uint16_t _purrOnTime = 4000,_purrOnTimeMin = 3000,_purrOnTimeMax = 4000;
  uint16_t _purrOffTime = 2000,_purrOffTimeMin = 1000,_purrOffTimeMax = 2500;
  Timer _purrTimer = Timer();

  // PANIC!
  bool _panicFlag = false;
  uint16_t _panicWiggleLeftDur = 250, _panicWiggleRightDur = 250;

  // Speaker Variables
  uint16_t _randPauseCall = 0, _randPauseCallMin = 1500, _randPauseCallMax = 2500;
  uint16_t _randPausePanic = 0, _randPausePanicMin = 100, _randPausePanicMax = 300;
  uint16_t _pauseInd = 7;
  uint16_t _callFreqs[8]  = {NOTE_C5,NOTE_C5,NOTE_C5,NOTE_C7,NOTE_E5,NOTE_C5,NOTE_C5,NOTE_C7};
  uint16_t _callDurs[8] = {250,0,200,500,250,0,200,2000};
  uint16_t _panicFreqs[8]  = {NOTE_C7,NOTE_C8,NOTE_C8,NOTE_C8,NOTE_C7,NOTE_C8,NOTE_C8,NOTE_C8};
  uint16_t _panicDurs[8] = {300,0,600,100,300,0,600,200};
};
#endif // PICKEDUP
