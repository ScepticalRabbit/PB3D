#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/UltrasonicSensor.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "UltrasonicSensor.h"

//------------------------------------------------------------------------------
// UPDATE: called during every LOOP
void UltrasonicSensor::update(){
    if(!_is_enabled){return;}

    _range = _ultrasonic_ranger.MeasureInCentimeters();
    if(_range <= _colDistLim){_range = 400;}
}


uint8_t UltrasonicSensor::get_collision_code(){
    if(_range <= _colDistClose){return DANGER_CLOSE;}
    else if(_range <= _colDistFar){return DANGER_FAR;}
    else if(_range <= _colDistSlowD){return DANGER_SLOW;}
    else{return DANGER_NONE;}
}

