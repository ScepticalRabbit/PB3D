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
    _timer.start(0);
}

//------------------------------------------------------------------------------
// UPDATE: called during every LOOP
void BumperSensor::update(){
    if(!enabled){return;}

    if(_timer.finished()){
        _timer.start(_update_time);

        // NOTE: need not here for the way the bumpers are wired
        _bumper_flags[BUMP_LEFT] = !_multi_expander->digital_read(
                                                    GPIO_BUMPER_LEFT);
        _bumper_flags[BUMP_RIGHT] = !_multi_expander->digital_read(
                                                    GPIO_BUMPER_RIGHT);
        _bumper_flags[BUMP_BACK] = !_multi_expander->digital_read(
                                                    GPIO_BUMPER_BACK);

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
}


EDangerCode BumperSensor::get_collision_code(EBumpCode bumpCode){
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
