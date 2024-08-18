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

class Timer{
public:
  Timer(){
  }

  void start(uint32_t timerDuration){
    _timer_start = millis();
    _timer_duration = timerDuration;
  }

  uint32_t get_time(){
    return (millis()-_timer_start);
  }

  bool finished(){
    return ((millis()-_timer_start)>_timer_duration);
  }

private:
  uint32_t _timer_start = 0;
  uint32_t _timer_duration = 0;
};
#endif // TIMER_H
