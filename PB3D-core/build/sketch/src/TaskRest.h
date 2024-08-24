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
#include "PB3DTimer.h"


class TaskRest{
public:
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
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;

  const uint8_t _rest_update_time = 7;
  Timer _timer = Timer();

  bool _rest_LED_increase = true;
  const uint8_t _rest_LED_max = 254;
  const uint8_t _rest_LED_min = 0;
  uint8_t _rest_LED_val = 254;
};
#endif // TASKREST
