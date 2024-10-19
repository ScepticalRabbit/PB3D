//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKINTERACT_H
#define TASKINTERACT_H

#include <Wire.h> // I2C
#include <Seeed_CY8C401XX.h> // Capcitive Touch Sensor

#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "TaskDance.h"
#include "Speaker.h"
#include "PB3DTimer.h"
#include "PatSensor.h"

class TaskInteract{
public:
  TaskInteract(MoodManager* mood, TaskManager* task, MoveManager* move,
               Speaker* speaker, TaskDance* dance, PatSensor* pat_sens);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // INTERACT - called during the main during decision tree
  void interact();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void set_start_interact_flag(bool start);

  bool get_enabled_flag(){return _enabled;}
  uint16_t get_timeout(){return _pat_timeout;}
  void set_enabled_flag(bool start){_enabled = start;}

private:
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  TaskDance* _task_dance = NULL;
  Speaker* _speaker = NULL;
  PatSensor* _pat_sensor = NULL;

  // TASK - INTERACT - Enabled Flag
  bool _enabled = true;

  bool _interact_start = true;

  bool _ask_start = false;
  const uint16_t _ask_squeak_interval = 7000;
  Timer _ask_squeak_timer = Timer();

  const uint16_t _ask_wiggle_left_dur = 500;
  const uint16_t _ask_wiggle_right_dur = 500;
  const uint16_t _askWiggleDuration =
                    2*(_ask_wiggle_left_dur+_ask_wiggle_right_dur);
  Timer _ask_wiggle_timer = Timer();

  const uint16_t _pat_timeout = 30000;
  Timer _pat_timeout_timer = Timer();
};
#endif
