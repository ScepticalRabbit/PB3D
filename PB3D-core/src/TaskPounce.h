//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKPOUNCE_H
#define TASKPOUNCE_H

#include <Wire.h> // I2C

#include <PB3DConstants.h>
#include "MoodManager.h"
#include "CollisionManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "PB3DTimer.h"
#include "Speaker.h"


class TaskPounce{
public:
  TaskPounce(CollisionManager* collision,
             MoodManager* mood,
             TaskManager* task,
             MoveManager* move,
             Speaker* speaker);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // Pounce! - called during the main during decision tree
  void seek_and_pounce();

  //---------------------------------------------------------------------------
  // Get, set and reset
  bool enabled = true;

  void reset();
  void collision_reset_to_realign();
  void set_realign_cent(int16_t angle);

  EPounce get_state(){return _state;}
  int16_t get_ang_cent_for_collision(){return _realign_ang_cent_collision;}


private:

  //---------------------------------------------------------------------------
  // START
  void _start();

  //---------------------------------------------------------------------------
  // SEEK
  void _seek_target();

  //---------------------------------------------------------------------------
  // LOCK
  void _lock_on();

  //---------------------------------------------------------------------------
  // RUN
  void _run_to_target();

  //---------------------------------------------------------------------------
  // REALIGN
  void _realign();

  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  CollisionManager* _collision_manager = NULL;
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES

  bool _start_all = true;

  EPounce _state = POUNCE_SEEK;
  EPounce _prev_state = POUNCE_REALIGN;

  // LED Colours
  uint8_t _seek_colour = 0;
  uint8_t _lock_colour = 2;
  uint8_t _run_colour = 4;
  uint8_t _realign_colour = 6;
  uint8_t _low_saturation = 128;

  // Seek
  bool _seek_start = true;
  Timer _meas_timer = Timer();
  uint16_t _meas_pre_pause_time = 150;
  uint16_t _meas_interval = 110;
  bool _meas_complete = false;

  uint8_t _meas_index = 0;
  static const uint8_t _meas_num_vals = 3;
  int16_t _meas_array[_meas_num_vals] = {-1,-1,-1};
  float _meas_angles[_meas_num_vals] = {30.0,0.0,-30.0};

  int16_t _meas_sum_for_avg = 0;
  int16_t _meas_count_for_avg = 0;

  // Lock On
  bool _lock_start = true;
  Timer _lock_on_timer = Timer();
  uint16_t _lock_spool_up_time = 800; // ms
  int16_t _lock_limit_range_min = 250;
  int16_t _lock_limit_range_max = 5000; // in mm
  int16_t _lock_valid_range_min = _lock_limit_range_max;
  int16_t _lock_valid_range_max = 0;
  uint8_t _lock_valid_range_min_ind = 0;
  uint8_t _lock_valid_range_max_ind = 0;
  uint8_t _lock_valid_range_count =  0;
  float _lock_on_ang = 0.0;
  float _lock_on_range = 0.0;

  // Run
  bool _run_start = true;
  Timer _run_timer = Timer();
  uint16_t _run_range_limit = 240; //mm
  int32_t _run_end_encoder_count = 0;
  float _run_speed = 300.0; //mmps
  uint16_t _run_timeout = 5000;

  // Realign
  bool _realign_start = true;
  int8_t _realign_state = 0;
  Timer _realign_timer = Timer();
  const uint16_t _realign_pre_pause_time = 1000;
  const uint16_t _realign_post_pause_time = 1000;
  const uint16_t _realignTimeout = 2000;

  int16_t _realign_angle_cent = 180;
  int16_t _realign_angle_dev = 45;
  int16_t _realign_ang_cent_collision = 60;
  int16_t _realign_ang_min = _realign_angle_cent-_realign_angle_dev;
  int16_t _realign_ang_max = _realign_angle_cent+_realign_angle_dev;
  float _realign_angle = 180.0;
};
#endif // END TASKPOUNCE
