#line 1 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/LaserManager.cpp"
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
#include "CollisionDangerCodes.h"

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void LaserManager::begin(){

    _laser_timer.start(0);

    if (!_gpio_expander.begin(ADDR_GPIO, &Wire)) {
        Serial.println("LSRMANAGER: Could not init PCF8574");
        while (1);
    }
    else {
        Serial.println("LSRMANAGER: init PCF8574 successful!");
    }
    for (uint8_t pp=0; pp<8; pp++) {
        _gpio_expander.pinMode(pp, OUTPUT);
    }

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<8; pp++) {
        _gpio_expander.digitalWrite(pp, LOW);
    }
    delay(_reset_delay);

    // Turn on all sensors - set all high
    for (uint8_t pp=0; pp<8; pp++) {
        _gpio_expander.digitalWrite(pp, HIGH);
    }
    delay(_reset_delay);

    // Reset all laser sensors - set all low
    for (uint8_t pp=0; pp<8; pp++) {
        _gpio_expander.digitalWrite(pp, LOW);
    }
    delay(_reset_delay);

    // Activate ALL lasers one by one
    for (uint8_t rr=0; rr<_num_lasers; rr++) {
        if (rr < 8){
            _gpio_expander.digitalWrite(rr, HIGH);
        }

        _laser_ptr_array[rr]->begin();
    }
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void LaserManager::update(){

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
EDangerCode LaserManager::get_collision_code(ELaserIndex _ind){
    return _laser_strategy_array[_ind]->get_collision_code(
        _laser_ptr_array[_ind]->get_range());
}

