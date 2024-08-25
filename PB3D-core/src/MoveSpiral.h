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
    MoveSpiral(MoveBasic* move_basic){
        _move_basic_code = move_basic;
    };

    void spiral();
    void spiral(int8_t turn_dir);
    void spiral_speed(int8_t turn_dir);
    void spiral_power(int8_t turn_dir);

private:
    MoveBasic* _move_basic_code = NULL;

    bool _spiral_start = true;
    uint32_t _spiral_start_time = 0;
    uint32_t _spiral_duration = 20000;
    uint32_t _spiral_curr_time = 0;
    int8_t _spiral_direction = MOVE_B_LEFT;

    float _spiral_min_speed = 5.0;
    float _spiral_slope = 0.0;
    float _init_spiral_speed_diff = _def_forward_speed-_min_speed;
    float _cur_spiral_speed_diff = 0.0;

    uint8_t _spiral_min_power = 5.0;
    float _spiral_slope_power = 0.0;
    uint8_t _init_spiral_speed_diff_power = _def_forward_power-_min_power;
    uint8_t _cur_spiral_speed_diff_power = 0;
};
#endif
