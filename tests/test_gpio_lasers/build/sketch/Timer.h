#line 1 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/Timer.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
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
  Timer();
  void start(uint32_t timerDuration);
  uint32_t getTime();
  bool finished();

private:
  uint32_t _timerStart = 0;
  uint32_t _timerDuration = 0;
};
#endif // TIMER_H
