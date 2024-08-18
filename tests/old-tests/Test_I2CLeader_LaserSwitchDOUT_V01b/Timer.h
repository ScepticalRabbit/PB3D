//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS - TIMER
//-----------------------------------------------------------------------------
/*
The timer class is part of the PetBot (PB) program. It used to time the
different functions of the robot

Author: Lloyd Fletcher
Date Created: 28th Aug. 2021
Date Edited:  28th Aug. 2021
*/
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
