//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_ZIGZAG_H
#define MOVE_ZIGZAG_H

#include <Arduino.h>

#include <PB3DConstants.h>
#include "MoveBasic.h"

class MoveZigZag{
public:
    MoveZigZag(MoveBasic* move_basic){
        _move_basic_code = move_basic;
    };

    void zig_zag();

private:
    MoveBasic* _move_basic_code = NULL;

    bool _turn_flag = true;
    uint16_t _init_turn_dur = 800;
    uint16_t _left_turn_dur = _init_turn_dur;
    uint16_t _right_turn_dur = _init_turn_dur;
    uint16_t _turn_duration = _left_turn_dur;

    bool _straight_flag = false;
    uint32_t _straight_duration = 1000;
    int8_t _turn_dir = MOVE_B_LEFT;

    uint8_t _turn_diff_power = round(_def_forward_power/2);
    float _turn_diff_speed = 0.5*_def_forward_speed;



};
#endif
