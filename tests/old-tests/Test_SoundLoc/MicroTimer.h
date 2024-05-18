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
    _timerStart = micros();
    _timerDuration = timerDuration;
  }

  uint32_t getTime(){
    return (micros()-_timerStart);
  }

  bool finished(){
    return ((micros()-_timerStart)>_timerDuration);
  }
  
private:
  uint32_t _timerStart = 0;
  uint32_t _timerDuration = 0;  
};
#endif // MICROTIMER_H
