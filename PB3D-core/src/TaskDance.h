//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKDANCE_H
#define TASKDANCE_H

#include <Arduino.h>

#include <PB3DConstants.h>

#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "PB3DTimer.h"


class TaskDance{
public:
  TaskDance(MoodManager* mood, TaskManager* task, MoveManager* move,
            Speaker* speaker);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every loop
  void update();

  //---------------------------------------------------------------------------
  // DANCE
  void dance();

  //---------------------------------------------------------------------------
  // Get, set and reset
  float get_dance_bar_ms(){return _dance_bar_ms;}
  uint32_t get_duration(){return _dance_duration;}
  void set_start_flag(bool flag){_dance_start = flag;}
  bool get_start_flag(){return _dance_start;}
  bool get_speaker_flag(){return _speaker_flag;}
  void set_speaker_flag(bool flag){_speaker_flag = flag;}

private:
  void _start_dance();
  void _generate_tempo();
  void _generate_dance();
  void _update_dance();

  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  Speaker* _speaker = NULL;


  Timer _timer = Timer();
  float _dance_bpm = 140.0;
  float _dance_4note_ms = (60.0/_dance_bpm)*1000.0;
  float _dance_bar_ms = _dance_4note_ms*4.0;
  uint8_t _dance_num_bars = 8;
  uint32_t _dance_duration = uint32_t(_dance_bar_ms*float(_dance_num_bars));

  int16_t _dance_bpm_min = 120;
  int16_t _dance_bpm_max = 180;
  uint8_t _dance_curr_move = 0;
  uint8_t _dance_curr_bar = 0;
  uint8_t _dance_move_ind = 0;
  uint8_t _dance_num_moves = 8;
  uint8_t _dance_move_vec[8] = {1,3,1,3,1,3,1,3};
  bool _dance_start = true;

  int8_t _dance_turn_dir = MOVE_B_LEFT;
  int8_t _dance_spin_dir = MOVE_B_LEFT;

  bool _speaker_flag = false;
};
#endif // TASKDANCE
