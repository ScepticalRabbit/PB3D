//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKFINDHUMAN_H
#define TASKFINDHUMAN_H

#include <Wire.h> // I2C
#include <Grove_Human_Presence_Sensor.h> // IR presence sensor

#include <PB3DI2CAddresses.h>
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "PB3DTimer.h"
#include "TaskInteract.h"


class TaskFindHuman{
public:
  TaskFindHuman(MoodManager* mood, TaskManager* task, MoveManager* move,
                Speaker* speaker, TaskInteract* task_int);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // FIND HUMAN - called during task decision tree
  void find_human();

  //---------------------------------------------------------------------------
  // Get, set and reset
  bool get_enabled_flag(){return _enabled;}
  void set_enabled_flag(bool flag){_enabled = flag;}

private:
  void _update_human_IR_sensor();

  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;
  TaskInteract* _task_interact = NULL;

  // Timers
  Timer _call_timer = Timer();

  // TASK - FIND HUMAN
  bool _enabled = true;
  bool _start_flag = false;

  // TASK - FIND HUMAN - Grove Human Sensor
  AK9753 _movement_sens = AK9753(ADDR_IR_PRES_SENS);
  const float _sensitivity_presence = 12.0;
  const float _sensitivity_movement = 11.5;
  const float _sensitivity_track = 2.9;
  const uint16_t _detect_interval = 30; //milliseconds
  PresenceDetector _pres_detector = PresenceDetector(_movement_sens,
                                                    _sensitivity_presence,
                                                    _sensitivity_movement,
                                                    _detect_interval);

  Timer _sens_update_timer = Timer();
  const uint8_t _IR_sens_update_time = 50;
  //NOTE: Sensors are organised clock wise with 1 closest to the grove connector
  bool _IR_detect1 = false;
  bool _IR_detect2 = false;
  bool _IR_detect3 = false;
  bool _IR_detect4 = false;
  float _IR_deriv1 = 0.0;
  float _IR_deriv2 = 0.0;
  float _IR_deriv3 = 0.0;
  float _IR_deriv4 = 0.0;
  float _diff_IR_1to3 = 0.0;
  float _diff_IR_2to4 = 0.0;

  // Variables for controlling calling: where you?
  const uint16_t _call_interval = 5000;
};
#endif // FINDHUMAN
