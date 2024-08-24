#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Navigation.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

#include "PB3DTimer.h"
#include "Encoder.h"
#include "IMUSensor.h"

//#define NAV_DEBUG_TIMER
//#define NAV_DEBUG_POS

class Navigation{
public:
  Navigation(Encoder* encoder_left, Encoder* encoder_right, IMUSensor* IMU);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // GET, SET, RESET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}

  float get_pos_x(){return _pos_x_next;}
  float get_pos_y(){return _pos_y_next;}
  float get_vel_x(){return _vel_x;}
  float get_vel_y(){return _vel_y;}
  float get_vel_c(){return _vel_c;}
  float get_heading(){return _heading;}
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;

  // Debug variables
  uint8_t _debug_count = 0;
  uint8_t _debug_freq = 10;

  // Navigation Objects - Encoders and IMU
  Encoder* _encoder_left;
  Encoder* _encoder_right;
  IMUSensor* _IMU;

  // Navigation Variables
  const int16_t _nav_update_time = 20;
  float _dt = float(_nav_update_time)/1000.0;
  Timer _nav_timer = Timer();

  float _vel_c = 0.0;
  float _vel_x = 0.0;
  float _vel_y = 0.0;
  float _heading = 0.0;
  float _pos_x_next = 0.0;
  float _pos_y_next = 0.0;
  float _pos_x_prev = 0.0;
  float _pos_y_prev = 0.0;
};
#endif //
