//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS: TIMER
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
