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
    _sensPatCountThres = random(_sensPatCountThresMin,_sensPatCountThresMax+1);
    // Start Timers
    _disableButtonsTimer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void PatSensor::update(){
    if(!_enabled){return;}

    // SENSOR: Check for button press unless disabled
    if(_buttonsEnabled){
        // 0x01 = b1, 0x02 = b2, 0x03 = b1+b2
        uint8_t valueButton = 0;
        _touchSens.get_touch_button_value(&valueButton);
        if(~_patFlag && _disableButtonsTimer.finished()){
        if(valueButton & 0x01){
            _patFlag = true;
            _buttonOneFlag = true;
        }
        else if(valueButton & 0x02){
            _patFlag = true;
            _buttonTwoFlag = true;
        }
        else{
            _buttonOneFlag = false;
            _buttonTwoFlag = false;
        }
        }
    }
    else{
        _buttonOneFlag = false;
        _buttonTwoFlag = false;
    }
}

//---------------------------------------------------------------------------
// ACCEPT PATS - called during the main during decision tree
void PatSensor::acceptPats(){
    if(!_enabled){return;}

    // Slider value, left=100, right=0
    uint8_t valueSlider = 0;
    _touchSens.get_touch_slider_value(&valueSlider);

    // See if we need to update the slider tolerance if it is in range
    if((valueSlider<=(_sensPatThres+_sensPatTol))&&(valueSlider>=(_sensPatThres-_sensPatTol))){
        _sensPatThres = _sensPatThres-_sensPatInc;
        if((_sensPatThres-_sensPatTol)<=0){
        // Reset the 'pat' state
        _sensPatThres = 100-_sensPatTol;
        _sensPatCount = _sensPatCount+1;
        }
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void PatSensor::reset(){
    _patFlag = false;   // Reset the pat flag
    _sensPatCount = 0;  // Reset the pat counter
    genPatCountThres(); // Generate a new threshold
    // Disable buttons once pat is over
    _disableButtonsTimer.start(_disableButtonsTime);
}

void PatSensor::genPatCountThres(){
    _sensPatCountThres = random(_sensPatCountThresMin,_sensPatCountThresMax+1);
}
