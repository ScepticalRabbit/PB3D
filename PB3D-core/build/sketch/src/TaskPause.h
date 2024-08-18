#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskPause.h"
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
  bool get_enabled_flag(){return _enabled;}
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

private:
  // MAIN OBJECT POINTERS
  CollisionManager* _collision_manager = NULL;
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;

  // TASK - PAUSE
  bool _enabled = true;
  Timer _pauseTimer = Timer();
  uint16_t _pauseDur = 2000;
  uint16_t _pauseDurMin = 2000, _pauseDurMax = 4000;
};
#endif // PAUSE
