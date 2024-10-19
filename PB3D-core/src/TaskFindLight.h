//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKFINDLIGHT_H
#define TASKFINDLIGHT_H

#include <Wire.h>
#include <Adafruit_VEML7700.h>

#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "PB3DTimer.h"
#include "Speaker.h"
#include "PatSensor.h"


class TaskFindLight{
public:
  TaskFindLight(MoodManager* mood, TaskManager* task, MoveManager* move,
                Speaker* speaker, PatSensor* pat_sens);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // FINDLIGHT - called during the main during decision tree
  void find_light();
  void find_dark();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset_gradient();
  bool get_enabled_flag(){return _enabled;}
  float get_lux_left(){return _lux_left;}
  float get_lux_right(){return _lux_right;}
  void set_enabled_flag(bool enabled){_enabled = enabled;}

private:
  void _find_lux(EFindLight seek_light);
  void _multiplex_select(uint8_t channel);

  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;
  PatSensor* _pat_sensor = NULL;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;

  Adafruit_VEML7700 _light_sens_left = Adafruit_VEML7700();
  Adafruit_VEML7700 _light_sens_right = Adafruit_VEML7700();

  const uint16_t _sens_update_time = 400;
  Timer _sense_timer = Timer();
  float _lux_left = 0.0;
  float _lux_right = 0.0;
  float _lux_avg = 0.0;
  float _lux_diff = 0.0;
  float _lux_percent_threshold_mid = 0.2;
  float _lux_threshold_mid = 0.0;
  float _lux_percent_thres_str = 0.5;
  float _lux_thres_str = 0.0;
  float _lux_thres_stop = 20.0;

  const uint16_t _gradient_update_time = 5000;
  Timer _gradient_timer = Timer();
  float _lux_avg_time0 = 0.0;
  float _lux_avg_time1 = 0.0;
  float _lux_time_avg = 0.0;
  float _lux_grad = 0.0;
  float _lux_grad_percent_thres = 0.2;
  float _lux_grad_thres = 0.0;
  bool _grad_move_flag = false;
  Timer _grad_move_timeout = Timer();
  const uint16_t _grad_move_timeout_time = 2000;
};

#endif
