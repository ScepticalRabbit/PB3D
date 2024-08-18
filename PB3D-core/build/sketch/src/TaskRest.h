#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskRest.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKREST_H
#define TASKREST_H

#include <Arduino.h>
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"

class TaskRest{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskRest(MoodManager* inMood, TaskManager* inTask,
            MoveManager* inMove, Speaker* inSpeaker);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // REST
  void rest();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset();

private:
  // MAIN OBJECT POINTERS
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;

  // Timers
  Timer _timerObj = Timer();

  // SUBTASK - REST Variables
  // Use to pulse LEDs during rest task
  uint8_t _restUpdateTime = 7;
  bool _restLEDIncrease = true;
  uint8_t _restLEDMax = 254;
  uint8_t _restLEDMin = 0;
  uint8_t _restLEDVal = 254;
};
#endif // TASKREST
