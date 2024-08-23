//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKFINDSOUND_H
#define TASKFINDSOUND_H

#include <Wire.h> // I2C

#include <PB3DI2CAddresses.h>
#include <PB3DConstants.h>
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"

// Debug flag to print debug info to serial
//#define DEBUG_TASKFINDSOUND

class TaskFindSound{
public:
  TaskFindSound(MoodManager* mood, TaskManager* task,
                MoveManager* move, Speaker* speaker);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during each loop to update variables
  void update();

  //---------------------------------------------------------------------------
  // FINDSOUND - called during task decision tree
  void find_sound();

  //---------------------------------------------------------------------------
  // GET/SET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}
  void set_enabled_flag(bool enabled){_enabled = enabled;}
  byte get_ear_state(){return _ear_state;}
  void set_ear_state(byte state){_ear_state = state;}
  byte get_send_byte(){return _send_byte;}
  void set_send_byte(byte in_byte){_send_byte = in_byte;}

private:
  void _I2C_send_byte();
  void _I2C_send_samp_env_flag();
  void _I2C_read_ear_state();

  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;

  bool _enabled = true;
  bool _start_flag = false;

  // TASK - FIND SOUND VARS
  byte _ear_state = 0;
  byte _send_byte = B00000000;
  float _speed_diff_leftright = 75.0;
  float _speed_diff_frac_leftright = 0.75;

  // Timer for controlling how often the sensor is sampled
  Timer _sens_update_timer = Timer();
  const uint16_t _sens_update_time = 500;

  Timer _clap_enable_timer = Timer();
  const uint16_t _clap_enable_update_time = 6000;
  const uint8_t _clap_thres = 3;
  uint8_t _clap_count = 0;

  Timer _env_samp_timer = Timer();
  const uint16_t _env_samp_update_time = 30000;

  // Variables for controlling calling: where are you?
  Timer _call_timer = Timer();
  const uint16_t _call_interval = 4000;
};
#endif // FINDSOUND
