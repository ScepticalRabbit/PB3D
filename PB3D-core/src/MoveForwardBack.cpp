//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveForwardBack.h"


void MoveForwardBack::forward_back(uint16_t forward_time, uint16_t back_time){
    if(_move_timer->finished()){
        _move_timer->start(_curr_dur);

         // Stop the motors because we are going to switch direction
        _move_basic->stop();
        if(_forward_flag){
            _forward_flag = false;
            _curr_dur = back_time;
        }
        else{
            _forward_flag = true;
            _curr_dur = forward_time;
        }
    }

    if(_forward_flag){
        _move_basic->forward();
    }
    else{
        _move_basic->back();
    }
}

void MoveForwardBack::forward_back(){
    forward_back(_def_forward_dur,_def_back_dur);
}
