//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKPAUSE_H
#define TASKPAUSE_H

#include <Wire.h> // I2C
#include "CollisionManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"

class TaskPause{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskPause(CollisionManager* inCollision, TaskManager* inTask,
            MoveManager* inMove, Speaker* inSpeaker);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // PAUSE
  void pause();

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

  // TASK - PAUSE
  bool _isEnabled = true;
  Timer _pauseTimer = Timer();
  uint16_t _pauseDur = 2000;
  uint16_t _pauseDurMin = 2000, _pauseDurMax = 4000;
};
#endif // PAUSE
