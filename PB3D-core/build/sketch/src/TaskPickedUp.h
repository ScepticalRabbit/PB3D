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
#include "PB3DTimer.h"
#include "Speaker.h"
#include "PatSensor.h"


class TaskPickedUp{
public:
  TaskPickedUp(CollisionManager* collision, MoodManager* mood,
               TaskManager* task, MoveManager* move, Speaker* speaker,
               PatSensor* pat_sens);

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
  bool get_enabled_flag(){return _enabled;}
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

private:
  // MAIN OBJECT POINTERS
  CollisionManager* _collision_manager = NULL;
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;
  PatSensor* _pat_sensor = NULL;

  // TASK - PICKED UP
  bool _enabled = true;
  bool _picked_up = false;
  bool _start_picked_up = false;
  bool _pat_flag = false;
  bool _pat_complete = false;

  bool _exit_flag = false;
  bool _exit_timer_on = false;
  const uint16_t _exit_time = 2000;
  Timer _exit_timer = Timer();

  // Call timer and update time
  uint16_t _call_update_time = 2000;
  Timer _call_timer = Timer();

  // Variables for purring
  byte _send_byte = B00001110;

  bool _purr_on = false;
  uint16_t _purr_on_time = 4000;
  const uint16_t _purr_on_time_min = 3000;
  const uint16_t _purr_on_time_max = 4000;
  uint16_t _purr_off_time = 2000;
  const uint16_t _purr_off_time_min = 1000;
  const uint16_t _purr_off_time_max = 2500;
  Timer _purr_timer = Timer();

  // PANIC!
  bool _panic_flag = false;
  const uint16_t _panic_wiggle_left_dur = 250;
  const uint16_t _panic_wiggle_right_dur = 250;

  // Speaker Variables
  uint16_t _rand_pause_call = 0;
  const uint16_t _rand_pause_call_min = 1500;
  const uint16_t _rand_pause_call_max = 2500;

  uint16_t _rand_pause_panic = 0;
  const uint16_t _rand_pause_panic_min = 100;
  const uint16_t _rand_pause_panic_max = 300;
  uint16_t _pause_ind = 7;

  uint16_t _call_freqs[8]  = {NOTE_C5,
                             NOTE_C5,
                             NOTE_C5,
                             NOTE_C7,
                             NOTE_E5,
                             NOTE_C5,
                             NOTE_C5,
                             NOTE_C7};
  uint16_t _call_durs[8] = {250,0,200,500,250,0,200,2000};

  uint16_t _panic_freqs[8]  = {NOTE_C7,
                              NOTE_C8,
                              NOTE_C8,
                              NOTE_C8,
                              NOTE_C7,
                              NOTE_C8,
                              NOTE_C8,
                              NOTE_C8};
  uint16_t _panic_durs[8] = {300,0,600,100,300,0,600,200};
};
#endif
