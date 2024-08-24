//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef PATSENSOR_H
#define PATSENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Seeed_CY8C401XX.h> // Capacitive Touch Sensor

#include <PB3DI2CAddresses.h>
#include "PB3DTimer.h"


class PatSensor{
public:
  PatSensor(){}

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // ACCEPT PATS - called during the main during decision tree
  void accept_pats();

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset();
  void gen_pat_count_thres();

  bool get_enabled_flag(){return _enabled;}
  bool get_pat_flag(){return _pat_flag;}
  bool get_button_one_flag(){return _button_one_flag;}
  bool get_button_two_flag(){return _button_two_flag;}
  uint8_t get_pat_count(){return _sens_pat_count;}
  bool get_pat_finished(){return (_sens_pat_count >= _sens_pat_count_thres);}

  void set_enabled_flag(bool enabled){_enabled = enabled;}
  void set_pat_flag(bool enabled){_pat_flag = enabled;}
  void set_pat_count_thres(uint8_t count){_sens_pat_count_thres = count;}
  void set_buttons_enabled(bool enabled){_buttons_enabled = enabled;}

private:
  CY8C _touch_sens = CY8C();

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;

  bool _pat_flag = false;
  bool _pat_complete = false;

  bool _button_one_flag = false;
  bool _button_two_flag = false;
  bool _buttons_enabled = true;

  uint8_t _sens_pat_count_thres = 3;
  const uint8_t _sens_pat_count_thres_min = 2;
  const uint8_t _sens_pat_count_thres_max = 5;
  uint8_t _sens_pat_count = 0;
  const int16_t _sens_pat_tol = 10;
  const int16_t _sens_pat_inc = 10;
  int16_t _sens_pat_thres = 100-_sens_pat_tol;

  // Timer to disable buttons after successful pat
  const uint16_t _disable_buttons_time = 5000;
  Timer _disable_buttons_timer = Timer();

};
#endif // CLASS PATSENSOR
