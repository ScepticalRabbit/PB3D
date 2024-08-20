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
#include <Wire.h> // I2C
//#include "CollisionManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Timer.h"
#include "Speaker.h"

#include <Servo.h>

#define TAIL_SERVO_POUT 8

#define TAIL_CENT 0
#define TAIL_SET_POS 1
#define TAIL_WAG_CON 2
#define TAIL_WAG_INT 3

class Tail{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  Tail(){
  }

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin(){
    // Servo - for wagging tail
    _tail_servo.attach(TAIL_SERVO_POUT);
    // Timer start
    _wag_timer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update(){
    if(!_enabled){return;}

    // Update the tail servo based on the main timer
    if(_update_timer.finished()){
      _update_timer.start(_update_interval);

      // Decision tree based on tail state - set by tasks+mood
      if(_curr_state == TAIL_SET_POS){
        _tail_servo.write(_tail_pos_curr);
      }
      else if(_curr_state == TAIL_WAG_CON){
        wag_continuous();
      }
      else if(_curr_state == TAIL_WAG_INT){
        wag_interval();
      }
      else{
        _tail_servo.write(_tail_pos_cent);
      }
    }
  }

  //---------------------------------------------------------------------------
  // TAIL FUNCTIONS
  void wag_continuous(){
    if(_wag_timer.finished()){
      _wag_timer.start(_wag_move_time);
      if(_wag_switch){
        _wag_switch = !_wag_switch;
        _tail_servo.write(_tail_pos_cent-_wag_pos_offset);
      }
      else{
        _wag_switch = !_wag_switch;
        _tail_servo.write(_tail_pos_cent+_wag_pos_offset);
      }
    }
  }

  void wag_interval(){
    if(_wag_timer.finished()){
      if(_wag_count<_wag_count_limit){
        _wag_timer.start(_wag_move_time);

        if(_wag_switch){
          _wag_switch = !_wag_switch;
          _tail_servo.write(_tail_pos_cent-_wag_pos_offset);
        }
        else{
          _wag_switch = !_wag_switch;
          _tail_servo.write(_tail_pos_cent+_wag_pos_offset);
          _wag_count++;
        }
      }
      else{
        _wag_timer.start(_wag_pause_time);
        _tail_servo.write(_tail_pos_cent);
        _wag_count = 0;
      }
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

  void set_state(uint8_t inState){_curr_state = inState;}
  void set_pos(int16_t inPos){_tail_pos_curr = inPos;}

  void set_wag_move_time(uint16_t inMoveTime){_wag_move_time = inMoveTime;}
  void set_wag_pos_offset(int16_t inOffset){_wag_pos_offset = inOffset;}
  void set_wag_pause_time(uint16_t inPauseTime){_wag_pause_time = inPauseTime;}
  void set_wag_count_lim(uint8_t inCountLim){_wag_count_limit = inCountLim;}

  void set_wag_params(uint16_t inMoveTime, int16_t inOffset, uint16_t inPauseTime, uint8_t inCountLim){
    _wag_move_time = inMoveTime;
    _wag_pos_offset = inOffset;
    _wag_pause_time = inPauseTime;
    _wag_count_limit = inCountLim;
  }

  void set_wag_con_params(uint16_t inMoveTime, int16_t inOffset){
    _wag_move_time = inMoveTime;
    _wag_pos_offset = inOffset;
  }

  void set_wag_int_params(uint16_t inMoveTime, int16_t inOffset, uint16_t inPauseTime, uint8_t inCountLim){
    _wag_move_time = inMoveTime;
    _wag_pos_offset = inOffset;
    _wag_pause_time = inPauseTime;
    _wag_count_limit = inCountLim;
  }

  void reset(){
    _curr_state = TAIL_CENT;
    _wag_count = 0;
  }

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;

  Servo _tail_servo;

  uint8_t _curr_state = TAIL_CENT;
  uint16_t _update_interval = 50;
  Timer _update_timer = Timer();

  // MODE: SET POS
  int16_t _tail_pos_curr = 90;
  int16_t _tail_pos_cent = 90;

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
#endif // END CLASS TAIL
