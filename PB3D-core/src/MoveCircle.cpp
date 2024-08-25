//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveCircle.h"

void MoveCircle::circle(){
    if(_turn_direction == MOVE_B_LEFT){
        _move_basic->forward_left();
    }
    else{
        _move_basic->forward_right();
    }
}
void MoveCircle::circle(int8_t turn_dir){
    if(turn_dir == MOVE_B_LEFT){
        _move_basic->forward_left();
    }
    else{
        _move_basic->forward_right();
    }
}
