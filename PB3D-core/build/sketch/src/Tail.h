#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Tail.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TAIL_H
#define TAIL_H

#include <Arduino.h>
#include <Wire.h>

#include <PB3DPins.h>
#include <PB3DConstants.h>
#include "TaskManager.h"
#include "MoveManager.h"
#include "Timer.h"
#include "Speaker.h"

#include <Servo.h>


class Tail{
public:
  Tail(){}

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // TAIL FUNCTIONS
  void wag_continuous();
  void wag_interval();

  //---------------------------------------------------------------------------
  // GET, SET, RESET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

  void set_state(ETailCode state){_curr_state = state;}
  void set_pos(int16_t pos){_tail_pos_curr = pos;}

  void set_wag_move_time(uint16_t move_time){_wag_move_time = move_time;}
  void set_wag_pos_offset(int16_t offset){_wag_pos_offset = offset;}
  void set_wag_pause_time(uint16_t pause_time){_wag_pause_time = pause_time;}
  void set_wag_count_lim(uint8_t count_limit){_wag_count_limit = count_limit;}

  void set_wag_params(uint16_t move_time,
                      int16_t offset,
                      uint16_t pause_time,
                      uint8_t count_limit);

  void set_wag_con_params(uint16_t move_time, int16_t offset);
  void set_wag_int_params(uint16_t move_time,
                          int16_t offset,
                          uint16_t pause_time,
                          uint8_t count_limit);

  void reset();

private:
  bool _enabled = true;

  Servo _tail_servo;

  uint8_t _curr_state = TAIL_CENT;
  const uint16_t _update_interval = 50;
  Timer _update_timer = Timer();

  int16_t _tail_pos_curr = 90;
  const int16_t _tail_pos_cent = 90;

  // MODE: WAG CONTINUOUS
  bool _wag_switch = true;
  int16_t _wag_pos_offset = 30;
  uint16_t _wag_move_time = 400;
  Timer _wag_timer = Timer();

  // MODE: WAG INTERVAL
  uint8_t _wag_count_limit = 4;
  uint8_t _wag_count = 0;
  uint16_t _wag_pause_time = 4000;
};
#endif // CLASS TAIL
