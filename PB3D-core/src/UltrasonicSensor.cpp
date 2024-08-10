//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "UltrasonicSensor.h"

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void UltrasonicSensor::update(){
    if(!_is_enabled){return;}

    _range = _ultrasonicRanger.MeasureInCentimeters();
    if(_range <= _colDistLim){_range = 400;}
}

//---------------------------------------------------------------------------
uint8_t UltrasonicSensor::get_collision_code(){
    if(_range <= _colDistClose){return DANGER_CLOSE;}
    else if(_range <= _colDistFar){return DANGER_FAR;}
    else if(_range <= _colDistSlowD){return DANGER_SLOWD;}
    else{return DANGER_NONE;}
}

