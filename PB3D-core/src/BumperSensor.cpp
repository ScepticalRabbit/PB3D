//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------
#include "BumperSensor.h"

//------------------------------------------------------------------------------
// BEGIN: called once during SETUP
void BumperSensor::begin(){
    // TODO: check that there is something at this address
    //Wire.requestFrom(ADDR_BUMPERS,1);
}

//------------------------------------------------------------------------------
// UPDATE: called during every LOOP
void BumperSensor::update(){
    if(!_is_enabled){return;}

    // Request a byte worth of digital pins from the follower Xiao
    Wire.requestFrom(ADDR_BUMPERS,1);
    // Read a byte from the follower
    byte bumperByte = B00000000;
    while(Wire.available()){
        bumperByte  = Wire.read();
    }
    _bumper_read_byte = bumperByte;

    if((_bumper_read_byte & _bumper_bytes[_bump_left])
        == _bumper_bytes[_bump_left]) {
        _bumper_flags[_bump_left] = true;
    }
    if ((_bumper_read_byte & _bumper_bytes[_bump_right])
        == _bumper_bytes[_bump_right]){
        _bumper_flags[_bump_right] = true;
    }

    // Loop over bumper flags to see if any are tripped
    for(uint8_t ii=0; ii<_num_bumpers; ii++){
        if(_bumper_flags[ii]){
            _bumper_any_flag = true;
        }
    }

    // If the bumpers are hit too many times decrease mood
    if(_bumper_any_flag){
        _bump_count++;
    }
}


uint8_t BumperSensor::get_collision_code(uint8_t bumpCode){
    if(bumpCode >= _num_bumpers){
        return DANGER_NONE;
    }
    if(_bumper_flags[bumpCode]){
        return DANGER_CLOSE;
    }
    else{
        return DANGER_NONE;
    }
}


void BumperSensor::reset(){
    _bumper_any_flag = false;
    for(uint8_t ii=0; ii<_num_bumpers; ii++){
        _bumper_flags[ii] = false;
    }
}
