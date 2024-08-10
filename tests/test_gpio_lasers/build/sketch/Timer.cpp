#line 1 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/Timer.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

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