//==============================================================================
// PB3D: A 3D printed pet robot
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
#include "PB3DTimer.h"

class TaskPause{
public:
  TaskPause(CollisionManager* collision, TaskManager* task,
            MoveManager* move, Speaker* speaker);

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
  bool enabled = true;

private:
  // MAIN OBJECT POINTERS
  CollisionManager* _collision_manager = NULL;
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;

  // TASK - PAUSE
  Timer _pause_timer = Timer();
  uint16_t _pause_dur = 2000;
  const uint16_t _pause_dur_min = 2000;
  const uint16_t _pause_dur_max = 4000;
};
#endif // PAUSE
