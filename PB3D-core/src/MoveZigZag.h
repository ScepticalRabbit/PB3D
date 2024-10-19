//==============================================================================
// PB3D: A 3D printed pet robot
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
#include <PB3DTimer.h>
#include "MoveBasic.h"


class MoveZigZag{
public:
    MoveZigZag(MoveBasic* move_basic,
               Timer* move_timer){
        _move_basic = move_basic;
        _move_timer = move_timer;
    }

    void zig_zag();

private:
    MoveBasic* _move_basic = NULL;
    Timer* _move_timer = NULL;

    EMoveZigZag _zig_zag_state = ZIGZAG_STRAIGHT;
    EMoveTurn _turn_dir = MOVE_TURN_LEFT;

    const uint16_t _init_turn_dur = 800;
    const uint32_t _straight_duration = 1000;
    const float _turn_diff_speed_frac = 0.5;
    uint16_t _left_turn_dur = _init_turn_dur;
    uint16_t _right_turn_dur = _init_turn_dur;
    uint16_t _turn_duration = _left_turn_dur;
};
#endif
