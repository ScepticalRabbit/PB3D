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
#include <Wire.h>

#include <PB3DConstants.h>
#include <PB3DI2CAddresses.h>
#include <PB3DPins.h>
#include <PB3DTimer.h>

#include "MultiIOExpander.h"


class BumperSensor{
public:
    BumperSensor(MultiIOExpander* multi_expander){
        _multi_expander = multi_expander;
    };

    //--------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin();

    //--------------------------------------------------------------------------
    // UPDATE: called during every LOOP
    void update();

    //---------------------------------------------------------------------------
    // Get, set and reset

    bool get_bump_flag(){return _bumper_any_flag;}
    bool get_bump_thres_check(){return (_bump_count >= _bump_thres);}
    int8_t get_bump_count(){return _bump_count;}
    void reset_bump_count(){_bump_count= 0;}

    EDangerCode get_collision_code(EBumpCode bump_code);

    void reset();

    bool enabled = true;

private:
    MultiIOExpander* _multi_expander = NULL;

    const static uint8_t _num_bumpers = 3;

    bool _bumper_any_flag = false;
    bool _bumper_flags[_num_bumpers] = {false,false,false};

    int8_t _bump_count = 0;
    const int8_t _bump_thres = 13;

    const int16_t _update_time = 51;
    Timer _timer = Timer();
};
#endif
