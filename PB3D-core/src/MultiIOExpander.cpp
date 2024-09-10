//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MultiIOExpander.h"

MultiIOExpander::MultiIOExpander(const uint8_t pin_modes[]){
    for(uint8_t pp=0; pp<total_pins; pp++){
        _pin_modes[pp] = pin_modes[pp];
    }
}

bool MultiIOExpander::begin(){
    if (!_gpio_expander_1.begin(ADDR_GPIO_1, &Wire)) {
        Serial.println("MULTI-IO: Could not init GPIO expander 1.");
        enabled = false;
        return false;
    }
    else {
        Serial.println("MULTI-IO: init GPIO expander 1 successful!");
    }

    if (!_gpio_expander_2.begin(ADDR_GPIO_2, &Wire)) {
        Serial.println("MULTI-IO: Could not init GPIO expander 2.");
        enabled = false;
        return false;
    }
    else {
        Serial.println("MULTI-IO: init GPIO expander 2 successful!");
    }

    uint8_t ee = 0;
    uint8_t exp_pin = 0;
    for(uint8_t pp=0; pp<total_pins; pp++){
        exp_pin = pp % pins_per_expander;

        _gpio_expanders[ee]->pinMode(exp_pin, _pin_modes[pp]);

        if(exp_pin >= (pins_per_expander-1) ){
            ee++;
        }
    }

    return true;
}

bool MultiIOExpander::digital_write(uint8_t pin, bool val){
    uint8_t io_exp = pin / pins_per_expander;
    uint8_t exp_pin = pin % pins_per_expander;
    return _gpio_expanders[io_exp]->digitalWrite(exp_pin,val);
}

bool MultiIOExpander::digital_read(uint8_t pin){
    uint8_t io_exp = pin / pins_per_expander;
    uint8_t exp_pin = pin % pins_per_expander;
    return _gpio_expanders[io_exp]->digitalRead(exp_pin);
}