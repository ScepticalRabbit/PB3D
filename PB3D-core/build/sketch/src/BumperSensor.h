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

#include "Timer.h"
#include "I2CAddress.h"
#include "CollisionDangerCodes.h"


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
    //---------------------------------------------------------------------------
    bool get_enabled_flag(){return _is_enabled;}
    void set_enabled_flag(bool inFlag){_is_enabled = inFlag;}

    bool get_bump_flag(){return _bumperAnyFlag;}
    bool get_bump_thres_check(){return (_bumpCount >= _bumpThres);}
    int8_t get_bump_count(){return _bumpCount;}
    void reset_bump_count(){_bumpCount= 0;}

    uint8_t get_collision_code(uint8_t bumpCode);
    void reset();

private:
    //---------------------------------------------------------------------------
    // CLASS VARIABLES
    //---------------------------------------------------------------------------
    bool _is_enabled = true;
    bool _start_flag = true;

    const static uint8_t _numBumpers = 2;
    const uint8_t _bumpLeft = 0;
    const uint8_t _bumpRight = 1;

    byte _bumperReadByte = B00000000;
    bool _bumperAnyFlag = false;
    bool _bumperFlags[_numBumpers] = {false,false};
    byte _bumperBytes[_numBumpers] = {B00000001,B00000010};

    int8_t _bumpCount = 0;
    int8_t _bumpThres = 13;
};
#endif
