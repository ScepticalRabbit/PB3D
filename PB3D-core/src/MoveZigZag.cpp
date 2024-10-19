//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveZigZag.h"


void MoveZigZag::zig_zag(){

    if(_zig_zag_state == ZIGZAG_TURN){
        if(!_move_timer->finished()){
            if(_turn_dir == MOVE_TURN_LEFT){
                _move_basic->forward_left_diff_frac(_turn_diff_speed_frac);

            }
            else{
                _move_basic->forward_right_diff_frac(_turn_diff_speed_frac);
            }
        }
        else{
            if(_turn_dir == MOVE_TURN_LEFT){
                _turn_dir = MOVE_TURN_RIGHT;
                _turn_duration = _right_turn_dur;
            }
            else{
                _turn_dir = MOVE_TURN_LEFT;
                _turn_duration = _left_turn_dur;
            }

            _zig_zag_state = ZIGZAG_STRAIGHT;
            _move_timer->start(_straight_duration);
        }
    }
    else if(_zig_zag_state == ZIGZAG_STRAIGHT){
        if(!_move_timer->finished()){
            _move_basic->forward();
        }
        else{
            _zig_zag_state = ZIGZAG_TURN;
            _move_timer->start(_turn_duration);
        }
    }
}