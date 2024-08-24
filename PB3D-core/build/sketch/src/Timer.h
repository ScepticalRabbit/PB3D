#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Timer.h"
//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS: TIMER
//-----------------------------------------------------------------------------
/*
The timer class is part of the PetBot (PB) program. It used to control the
timing of different functions of the robot.

Author: Lloyd Fletcher
*/
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
