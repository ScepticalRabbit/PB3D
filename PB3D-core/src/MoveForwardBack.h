//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_FORWARD_BACK_H
#define MOVE_FORWARD_BACK_H

#include <Arduino.h>
#include "MoveBasic.h"

class MoveForwardBack{
public:
    MoveForwardBack(MoveBasic* move_basic, Timer* move_timer){
        _move_basic = move_basic;
        _move_timer = move_timer;
    };

    void forward_back(uint16_t forward_time, uint16_t back_time);
    void forward_back();

private:
    MoveBasic* _move_basic = NULL;
    Timer* _move_timer = NULL;

    bool _forward_flag = true;
    const uint16_t _def_forward_dur = 500;
    const uint16_t _def_back_dur = 500;
    uint16_t _curr_dur = _def_forward_dur;
};
#endif
