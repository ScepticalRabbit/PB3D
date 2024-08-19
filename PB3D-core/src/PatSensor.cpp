//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "PatSensor.h"

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void PatSensor::begin(){
    // Ping the touch sensor to see if it is connected
    Wire.beginTransmission(ADDR_TOUCHSENS);
    if(Wire.endTransmission() != 0){
        Serial.println(F("PATSENSOR: Failed to initialise touch sensor."));
        _enabled = false;
    }
    else{
        Serial.println(F("PATSENSOR: Initialised touch sensor."));
        _enabled = true;
    }

    // Generate Random Numbers
    _sens_pat_count_thres = random(_sens_pat_count_thres_min,_sens_pat_count_thres_max+1);
    // Start Timers
    _disable_buttons_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void PatSensor::update(){
    if(!_enabled){return;}

    // SENSOR: Check for button press unless disabled
    if(_buttons_enabled){
        // 0x01 = b1, 0x02 = b2, 0x03 = b1+b2
        uint8_t valueButton = 0;
        _touch_sens.get_touch_button_value(&valueButton);
        if(~_pat_flag && _disable_buttons_timer.finished()){
        if(valueButton & 0x01){
            _pat_flag = true;
            _button_one_flag = true;
        }
        else if(valueButton & 0x02){
            _pat_flag = true;
            _button_two_flag = true;
        }
        else{
            _button_one_flag = false;
            _button_two_flag = false;
        }
        }
    }
    else{
        _button_one_flag = false;
        _button_two_flag = false;
    }
}

//---------------------------------------------------------------------------
// ACCEPT PATS - called during the main during decision tree
void PatSensor::accept_pats(){
    if(!_enabled){return;}

    // Slider value, left=100, right=0
    uint8_t valueSlider = 0;
    _touch_sens.get_touch_slider_value(&valueSlider);

    // See if we need to update the slider tolerance if it is in range
    if((valueSlider<=(_sens_pat_thres+_sens_pat_tol))&&(valueSlider>=(_sens_pat_thres-_sens_pat_tol))){
        _sens_pat_thres = _sens_pat_thres-_sens_pat_inc;
        if((_sens_pat_thres-_sens_pat_tol)<=0){
        // Reset the 'pat' state
        _sens_pat_thres = 100-_sens_pat_tol;
        _sens_pat_count = _sens_pat_count+1;
        }
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void PatSensor::reset(){
    _pat_flag = false;   // Reset the pat flag
    _sens_pat_count = 0;  // Reset the pat counter
    gen_pat_count_thres(); // Generate a new threshold
    // Disable buttons once pat is over
    _disable_buttons_timer.start(_disable_buttons_time);
}

void PatSensor::gen_pat_count_thres(){
    _sens_pat_count_thres = random(_sens_pat_count_thres_min,_sens_pat_count_thres_max+1);
}
