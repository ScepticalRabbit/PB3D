//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKPOUNCE_H
#define TASKPOUNCE_H

#include <Wire.h> // I2C
#include "MoodManager.h"
#include "CollisionManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Timer.h"
#include "Speaker.h"

#define POUNCE_SEEK 0
#define POUNCE_LOCKON 1
#define POUNCE_RUN 2
#define POUNCE_REALIGN 3

class TaskPounce{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskPounce(CollisionManager* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
             Speaker* inSpeaker);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // Pounce! - called during the main during decision tree
  void seekAndPounce();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset();
  void collisionResetToRealign();
  void setRealignCent(int16_t inAng);

  bool get_enabled_flag(){return _is_enabled;}
  int8_t getState(){return _state;}
  int16_t getAngCentForCollision(){return _realignAngCentCollision;}
  void set_enabled_flag(bool inFlag){_is_enabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS

  //---------------------------------------------------------------------------
  // START
  void _startAll();

  //---------------------------------------------------------------------------
  // SEEK
  void _seekTarget();

  //---------------------------------------------------------------------------
  // LOCK
  void _lockOn();

  //---------------------------------------------------------------------------
  // RUN
  void _runToTarget();

  //---------------------------------------------------------------------------
  // REALIGN
  void _realign();

  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  CollisionManager* _collisionObj = NULL;
  MoodManager* _moodObj = NULL;
  TaskManager* _taskObj = NULL;
  MoveManager* _moveObj = NULL;
  Speaker* _speakerObj = NULL;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _is_enabled = true;
  bool _startAllFlag = true;

  // Overall state
  int8_t _state = POUNCE_SEEK;
  int8_t _prevState = POUNCE_REALIGN;

  // LED Colours
  uint8_t _seekCol = 0;
  uint8_t _lockCol = 2;
  uint8_t _runCol = 4;
  uint8_t _realignCol = 6;
  uint8_t _lowSat = 128;

  // Seek
  bool _seekStart = true;
  Timer _measTimer = Timer();
  uint16_t _measPrePauseTime = 150;
  uint16_t _measInterval = 110;
  bool _measFlag = false;

  uint8_t _measCurInd = 0;
  uint8_t _measNumVals = 3;
  int16_t _measVec[3] = {-1,-1,-1};
  float _measAngs[3] = {30.0,0.0,-30.0};

  int16_t _measSumForAvg = 0;
  int16_t _measNumForAvg = 3;
  int16_t _measCountForAvg = 0;

  // Lock On
  bool _lockStartFlag = true;
  Timer _lockOnTimer = Timer();
  uint16_t _lockSpoolUpTime = 800; // ms
  int16_t _lockLimRangeMin = 250, _lockLimRangeMax = 5000; // in mm
  int16_t _lockValidRangeMin = _lockLimRangeMax, _lockValidRangeMax = 0;
  uint8_t _lockValidRangeMinInd = 0, _lockValidRangeMaxInd = 0;
  uint8_t _lockValidRangeCount =  0;
  float _lockOnAng = 0.0, _lockOnRange = 0.0;

  // Run
  bool _runStartFlag = true;
  Timer _runTimer = Timer();
  uint16_t _runRangeLim = 240; //mm
  int32_t _runEndEncCount = 0;
  float _runSpeed = 300.0; //mmps
  uint16_t _runTimeout = 5000;

  // Realign
  bool _realignStartFlag = true;
  int8_t _realignState = 0;
  Timer _realignTimer = Timer();
  uint16_t _realignPrePauseTime = 1000, _realignPostPauseTime = 1000;
  uint16_t _realignTimeout = 2000;

  int16_t _realignAngCent = 180, _realignAngDev = 45;
  int16_t _realignAngCentCollision = 60;
  int16_t _realignAngMin = _realignAngCent-_realignAngDev;
  int16_t _realignAngMax = _realignAngCent+_realignAngDev;
  float _realignAng = 180.0;
};
#endif // END TASKPOUNCE
