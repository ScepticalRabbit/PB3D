//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveWiggle.h"

void MoveWiggle::wiggle(uint16_t left_time, uint16_t right_time){
        if(_move_timer->finished()){
        _move_basic->stop(); // Stop the motors because we are going to switch direction
        if(_wiggle_left_flag){
            _wiggle_left_flag = false;
            _wiggle_curr_dur =  left_time;
        }
        else{
            _wiggle_left_flag = true;
            _wiggle_curr_dur = right_time;
        }
        _move_timer->start(_wiggle_curr_dur);
    }

    if(_wiggle_left_flag){
        _move_basic->left();
    }
    else{
        _move_basic->right();
    }
}

void MoveWiggle::wiggle(){
    wiggle(_wiggle_left_dur,_wiggle_right_dur);
}
