//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS - MICRO TIMER
//-----------------------------------------------------------------------------
/*
TODO

Author: Lloyd Fletcher
Date Created: 7th Nov. 2021
Date Edited:  7th Nov. 2021
*/
#ifndef MICROTIMER_H
#define MICROTIMER_H

class MicroTimer{
public:
  MicroTimer(){
  }

  void start(uint32_t timerDuration){
    _timer_start = micros();
    _timer_duration = timerDuration;
  }

  uint32_t get_time(){
    return (micros()-_timer_start);
  }

  bool finished(){
    return ((micros()-_timer_start)>_timer_duration);
  }

private:
  uint32_t _timer_start = 0;
  uint32_t _timer_duration = 0;
};
#endif // MICROTIMER_H
