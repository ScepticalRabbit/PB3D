//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef FILTERLOWPASS_H
#define FILTERLOWPASS_H

#include <Arduino.h>

#include "PB3DTimer.h"


class FilterLowPass{
public:
  FilterLowPass();
  FilterLowPass(double alpha);
  FilterLowPass(double alpha, uint16_t update_time);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  double filter(double data);

  void set_alpha(double alpha){_alpha = alpha;}
  double get_alpha(){return _alpha;}
  double get_current_value(){return _curr_filtered;}
  void reset(){_prev_filtered = 0.0;}
  void reset(double val){_prev_filtered = val;}

private:
  Timer _filter_timer;
  uint16_t _update_time = 1; // Default update time is 1kHz
  double _alpha = 0.1;
  double _prev_filtered = 0.0;
  double _curr_filtered = 0.0;
};
#endif // FILTERLOWPASS_H
