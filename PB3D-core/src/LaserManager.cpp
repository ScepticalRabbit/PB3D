//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include <Arduino.h>
#include <Adafruit_PCF8574.h>
#include "LaserManager.h"

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void LaserManager::begin(){
    if(!enabled){return;}

    _laser_timer.start(0);

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<LASER_COUNT; pp++) {
        _multi_expander->digital_write(pp, LOW);
    }
    delay(_reset_delay);

    // Turn on all sensors - set all high
    for (uint8_t pp=0; pp<LASER_COUNT; pp++) {
        _multi_expander->digital_write(pp, HIGH);
    }
    delay(_reset_delay);

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<LASER_COUNT; pp++) {
        _multi_expander->digital_write(pp, LOW);
    }
    delay(_reset_delay);

    // Activate ALL lasers one by one
    for (uint8_t rr=0; rr<LASER_COUNT; rr++) {
        if (rr < 8){
            _multi_expander->digital_write(rr, HIGH);
        }

        _laser_ptr_array[rr]->begin();
    }
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void LaserManager::update(){
    if(!enabled){return;}

    bool timer_latch = false;
    if(_laser_timer.finished()){
        _laser_timer.start(_laser_update_time);
        timer_latch = true;
    }

    for(uint8_t ll=0; ll < _num_lasers; ll++){
        if(_laser_ptr_array[ll]->get_enabled()){

            if(timer_latch){
                _laser_ptr_array[ll]->start_range();

            }

            if(_laser_ptr_array[ll]->update_range()){
                #ifdef DEBUG_LSRMANAGER
                // TODO: write serial print debugs
                #endif
            }
        }
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
EDangerCode LaserManager::get_collision_code(ELaserIndex index){

    EDangerCode danger_code = DANGER_NONE;
    switch(_laser_strategy_array[index]) {
        case AVOID_BASIC:
            danger_code = _avoid_basic.get_collision_code(
                            _laser_ptr_array[index]->get_range());
            break;
        case AVOID_FLAT_SLOW:
            danger_code = _avoid_flat_slow.get_collision_code(
                            _laser_ptr_array[index]->get_range());
            break;
        case AVOID_OVERHEAD:
            danger_code = _avoid_overhead.get_collision_code(
                            _laser_ptr_array[index]->get_range());
            break;
        case AVOID_CLIFF:
            danger_code = _avoid_cliff.get_collision_code(
                            _laser_ptr_array[index]->get_range());
            break;
        case AVOID_PICKUP:
            danger_code = _avoid_pickup.get_collision_code(
                            _laser_ptr_array[index]->get_range());
            break;
        default:
            danger_code = DANGER_NONE;
            break;
    }

    return danger_code;
}

