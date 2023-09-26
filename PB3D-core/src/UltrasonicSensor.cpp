//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: UltrasonicSensor
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "UltrasonicSensor.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//---------------------------------------------------------------------------
UltrasonicSensor::UltrasonicSensor(){

}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
//---------------------------------------------------------------------------
void UltrasonicSensor::begin(){

}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void UltrasonicSensor::update(){
    if(!_isEnabled){return;}

    _range = _ultrasonicRanger.MeasureInCentimeters();
    if(_range <= _colDistLim){_range = 400;}
}

//---------------------------------------------------------------------------
uint8_t UltrasonicSensor::getColCode(){
    if(_range <= _colDistClose){return DANGER_CLOSE;}
    else if(_range <= _colDistFar){return DANGER_FAR;}
    else if(_range <= _colDistSlowD){return DANGER_SLOWD;}
    else{return DANGER_NONE;}
}

