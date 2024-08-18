//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>
#include <Wire.h>       // I2C
#include <Ultrasonic.h> // Grove ultrasonic sensor

#include "PB3DConstants.h"

#define COLL_USSENS 7

//------------------------------------------------------------------------------
// CLASS: UltrasonicSensor
//------------------------------------------------------------------------------
class UltrasonicSensor{
public:
    UltrasonicSensor(){};

    //--------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin(){};

    //--------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    void update();

    //--------------------------------------------------------------------------
    // Get, set and reset
    bool get_enabled_flag(){return _enabled;}
    void set_enabled_flag(bool inFlag){_enabled = inFlag;}

    int16_t get_range(){return _range;}
    int16_t get_range_mm(){return _range*10;}

    uint8_t get_collision_code();

private:
    //--------------------------------------------------------------------------
    // CLASS VARIABLES
    //---------------------------------------------------------------------------
    bool _enabled = true;
    bool _start_flag = true;

    Ultrasonic _ultrasonic_ranger = Ultrasonic(COLL_USSENS);

    uint16_t _half_body_leng_mm = 80;
    int16_t _range = 2000;
    int16_t _colDistClose = _half_body_leng_mm/10; // cm
    int16_t _colDistFar = 2*(_half_body_leng_mm/10);  // cm
    int16_t _colDistSlowD = 3*(_half_body_leng_mm/10); // cm
    int16_t _colDistLim = 4;    // cm
};
#endif
