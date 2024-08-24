#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/FilterMovAvg.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef FILTERMOVAVG_H
#define FILTERMOVAVG_H

#include <Arduino.h>

#include "PB3DTimer.h"


class FilterMovAvg{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR/DESTRUCTOR
  FilterMovAvg();
  FilterMovAvg(uint8_t window);
  FilterMovAvg(uint8_t window, uint16_t update_time);
  ~FilterMovAvg();

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // FILTER
  double filter(double data);

  //---------------------------------------------------------------------------
  // Get, set and reset
  void reset();
  void reset(float val);

  uint8_t get_window(){return _window;}
  double get_current_value(){return _curr_filtered;}

private:
  Timer _filter_timer;
  uint16_t _update_time = 1; // Default update time is 1ms (1kHz)
  uint8_t _window = 5;      // Default window is 5 data points
  uint8_t _currIndex = 0;
  double _curr_filtered = 0.0;
  double _dataSum = 0.0;
  double* _dataArray;
};
#endif // FILTERMOVAVG_H
