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
    _timerStart = millis();
    _timerDuration = timerDuration;
  }

  uint32_t getTime(){
    return (millis()-_timerStart);
  }

  bool finished(){
    return ((millis()-_timerStart)>_timerDuration);
  }

private:
  uint32_t _timerStart = 0;
  uint32_t _timerDuration = 0;
};
#endif // TIMER_H
