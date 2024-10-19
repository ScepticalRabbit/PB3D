//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_SPIRAL_H
#define MOVE_SPIRAL_H

#include <Arduino.h>

#include <PB3DConstants.h>

#include "MoveBasic.h"

class MoveSpiral{
public:
    MoveSpiral(MoveBasic* move_basic,
               Timer* move_timer){
        _move_basic = move_basic;
        _move_timer = move_timer;
    }

    void spiral();
    void spiral(EMoveTurn turn_dir);

    void reset();

    EMoveTurn turn_direction = MOVE_TURN_LEFT;
    uint32_t duration = 20000;

private:
    MoveBasic* _move_basic = NULL;
    Timer* _move_timer = NULL;

    bool _spiral_start = true;
    uint32_t _spiral_start_time = 0;
    uint32_t _spiral_curr_time = 0;

    float _spiral_min_speed = 5.0;
    float _spiral_slope = 0.0;
    float _init_spiral_speed_diff = 0.0;
    float _cur_spiral_speed_diff = 0.0;
};
#endif
