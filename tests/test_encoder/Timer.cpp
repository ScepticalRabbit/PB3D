//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: TIMER
//-----------------------------------------------------------------------------
/*
The timer class is part of the PetBot (PB) program. It used to control the
timing of different functions of the robot.

Author: Lloyd Fletcher
*/
#include "PB3DTimer.h"

Timer::Timer(){

}

void Timer::start(uint32_t timerDuration){
    _timer_start = millis();
    _timer_duration = timerDuration;
}

uint32_t Timer::get_time(){
    return (millis()-_timer_start);
}

bool Timer::finished(){
    return ((millis()-_timer_start)>_timer_duration);
}