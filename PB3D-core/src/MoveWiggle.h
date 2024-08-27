//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_WIGGLE_H
#define MOVE_WIGGLE_H

#include <Arduino.h>
#include "MoveBasic.h"

class MoveWiggle{
public:
    MoveWiggle(MoveBasic* move_basic, Timer* move_timer){
        _move_basic = move_basic;
        _move_timer = move_timer;
    };

    void wiggle(uint16_t left_time, uint16_t right_time);
    void wiggle();

private:
    MoveBasic* _move_basic = NULL;
    Timer* _move_timer = NULL;

    bool _wiggle_left_flag = true;
    const uint16_t _wiggle_left_dur = 600;
    const uint16_t _wiggle_right_dur = 600;
    uint16_t _wiggle_curr_dur = _wiggle_left_dur;
};
#endif
