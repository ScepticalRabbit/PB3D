//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveCircle.h"

void MoveCircle::circle(){
    if(turn_direction == MOVE_TURN_LEFT){
        _move_basic->forward_left();
    }
    else{
        _move_basic->forward_right();
    }
}
void MoveCircle::circle(EMoveTurn turn_dir){
    if(turn_dir == MOVE_TURN_LEFT){
        _move_basic->forward_left();
    }
    else{
        _move_basic->forward_right();
    }
}
