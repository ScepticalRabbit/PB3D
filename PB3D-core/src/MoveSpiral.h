//==============================================================================
// PB3D: A pet robot that is 3D printed
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
    MoveSpiral(MoveBasic* move_basic, Timer* move_timer){
        _move_basic = move_basic;
        _move_timer = move_timer;
    }

    void spiral();
    void spiral(EMoveBasic turn_dir);

    void set_turn_direction(EMoveBasic turn_dir){_spiral_direction = turn_dir;}
    void reset();

private:
    MoveBasic* _move_basic = NULL;
    Timer* _move_timer = NULL;

    bool _spiral_start = true;
    uint32_t _spiral_start_time = 0;
    uint32_t _spiral_duration = 20000;
    uint32_t _spiral_curr_time = 0;
    EMoveBasic _spiral_direction = MOVE_B_LEFT;

    float _spiral_min_speed = 5.0;
    float _spiral_slope = 0.0;
    float _init_spiral_speed_diff = 0.0;
    float _cur_spiral_speed_diff = 0.0;

};
#endif
