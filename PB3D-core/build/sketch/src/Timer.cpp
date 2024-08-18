#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Timer.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "Timer.h"


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