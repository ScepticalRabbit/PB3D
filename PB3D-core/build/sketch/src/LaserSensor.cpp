#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/LaserSensor.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "LaserSensor.h"

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
//---------------------------------------------------------------------------
void LaserSensor::begin(){
    delay(_reset_delay);
    if(!_laser_obj.begin(_address)){
        Serial.print(F("COLLISION: FAILED to init laser "));
        Serial.println(_laser_ind);
        _enabled = false;
    }
    else{
        Serial.print(F("COLLISION: initialised laser "));
        Serial.println(_laser_ind);
        _enabled = true;
    }
    delay(_reset_delay);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void LaserSensor::start_range(){
    if(!_enabled){return;}

    _laser_obj.startRange();
    _range_start_time = millis();
    _range_flag = false;
}

bool LaserSensor::update_range(){
    if(!_enabled){return false;}

    if(_laser_obj.isRangeComplete() && !_range_flag){
        _range_timeout = _laser_obj.timeoutOccurred();
        _range_status = _laser_obj.readRangeStatus();
        _range = _laser_obj.readRangeResult();
        _range_flag = true;

        // Check failure conditions for valid ranging, failure range = -1
        if(_range_timeout){
            _range = -1;
        }
        if(_range_status != 0){
            _range = -1;
        }
        if(_range <= _range_limit){
            _range = -1;
        }

        _range_time = millis()-_range_start_time;
        return true;
    }
    else{
        return false;
    }
}
