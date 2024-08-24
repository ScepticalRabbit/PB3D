#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskTantrum.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------
#ifndef TASKTANTRUM_H
#define TASKTANTRUM_H

#include <Arduino.h>
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "PB3DTimer.h"

class TaskTantrum{
public:
  TaskTantrum(MoodManager* mood, TaskManager* task,
              MoveManager* move, Speaker* speaker);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // TANTRUM
  void chuck_tantrum();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void set_start_tantrum();

  uint8_t get_threshold(){return _tantrum_threshold;}
  uint16_t get_duration(){return _tantrum_duration;}
  bool get_complete(){return _tantrum_complete;}

private:
  // MAIN OBJECT POINTERS
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;
  // Timers
  Timer _tantrum_timer = Timer();
  Timer _growl_timer = Timer();

  // SUBTASK - TANTRUM Variables
  bool _start_tantrum = false;
  bool _tantrum_complete = false;
  const uint16_t _tantrum_threshold = 31;
  const uint16_t _tantrum_FB_duration = 400;
  const uint8_t _tantrum_FB_num = 5;
  const uint16_t _tantrum_duration = 2*_tantrum_FB_num*_tantrum_FB_duration;

  bool _growl_on = true;
  const uint16_t _tantrum_growl_duration = 2*_tantrum_FB_duration;
  const uint16_t _tantrum_growl_pause = 1*_tantrum_FB_duration;
};
#endif
