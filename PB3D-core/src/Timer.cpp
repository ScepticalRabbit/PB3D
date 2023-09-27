//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS: TIMER
//-----------------------------------------------------------------------------
/*
The timer class is part of the PetBot (PB) program. It used to control the
timing of different functions of the robot.

Author: Lloyd Fletcher
*/
#include "Timer.h"

Timer::Timer(){

}

void Timer::start(uint32_t timerDuration){
    _timerStart = millis();
    _timerDuration = timerDuration;
}

uint32_t Timer::getTime(){
    return (millis()-_timerStart);
}

bool Timer::finished(){
    return ((millis()-_timerStart)>_timerDuration);
}