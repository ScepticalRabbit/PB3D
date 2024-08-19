#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/BumperSensor.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------
#ifndef BUMPERSENSOR_H
#define BUMPERSENSOR_H

#include <Arduino.h>
#include <Wire.h> // I2C

#include "PB3DConstants.h"
#include "PB3DI2CAddresses.h"
#include "Timer.h"


class BumperSensor{
public:
    BumperSensor(){}

    //--------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin();

    //--------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    void update();

    //---------------------------------------------------------------------------
    // Get, set and reset
    bool get_enabled_flag(){return _enabled;}
    void set_enabled_flag(bool inFlag){_enabled = inFlag;}

    bool get_bump_flag(){return _bumper_any_flag;}
    bool get_bump_thres_check(){return (_bump_count >= _bump_thres);}
    int8_t get_bump_count(){return _bump_count;}
    void reset_bump_count(){_bump_count= 0;}

    EDangerCode get_collision_code(EBumpCode bump_code);

    void reset();

private:
    bool _enabled = true;
    bool _start_flag = true;

    const static uint8_t _num_bumpers = 2;
    const uint8_t _bump_left = 0;
    const uint8_t _bump_right = 1;

    byte _bumper_read_byte = B00000000;
    bool _bumper_any_flag = false;
    bool _bumper_flags[_num_bumpers] = {false,false};
    const byte _bumper_bytes[_num_bumpers] = {B00000001,B00000010};

    int8_t _bump_count = 0;
    const int8_t _bump_thres = 13;
};
#endif
