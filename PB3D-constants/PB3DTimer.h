//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

class Timer{
public:
  Timer(){};

  void start(uint32_t timer_duration);
  uint32_t get_time();
  bool finished();

private:
  uint32_t _timer_start = 0;
  uint32_t _timer_duration = 0;
};
#endif // TIMER_H
