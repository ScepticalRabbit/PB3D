//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveSpiral.h"

void MoveSpiral::spiral(){
    spiral(_spiral_direction);
}


void MoveSpiral::spiral(EMoveBasic turnDir){
    if(_spiral_start){
        _spiral_start = false;

        // Calculate the speed/time slope for the linear increase of speed
        // for the slow wheel
        _init_spiral_speed_diff = _move_basic->get_current_speed()-_spiral_min_speed;
        _cur_spiral_speed_diff = _init_spiral_speed_diff;
        _spiral_slope = _init_spiral_speed_diff/float(_spiral_duration);

        _move_timer->start(_spiral_duration);
    }

    // Calculate the current speed for the slope wheel based on the timer
    _spiral_curr_time = _move_timer->get_time();
    _cur_spiral_speed_diff = _init_spiral_speed_diff - _spiral_slope*float(_spiral_curr_time);

    // Check if we are increasing the speed of the slow wheel above the fast one
    if(_cur_spiral_speed_diff>_move_basic->get_current_speed()){
        _cur_spiral_speed_diff = _move_basic->get_current_speed()-_min_speed;
    }

    if(_move_timer->finished()){
        _spiral_start = true;
    }
    else{
        if(turnDir == MOVE_B_LEFT){
            _move_basic->forward_left_speed(_move_basic->get_current_speed(),
                                           _cur_spiral_speed_diff);
        }
        else{
            _move_basic->forward_right_speed(_move_basic->get_current_speed(),
                                            _cur_spiral_speed_diff);
        }
    }
}
