//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveLook.h"

EMoveLookState MoveLook::look_around(){
    if(_look_state == MOVE_LOOK_START){
        _look_state = MOVE_LOOK_MOVING;

        _look_timer.start(_look_move_time);
        _look_angle_index = 0;
        _move_controller->reset();
    }

    if(_look_timer.finished()){
        _move_controller->reset();

        if(_look_state == MOVE_LOOK_PAUSE){
            _look_state = MOVE_LOOK_MOVING;
            _look_timer.start(_look_move_time);
            _look_angle_index++;

            if(_look_angle_index > _look_num_angs){
                _look_angle_index = 0;
            }
        }
        else if(_look_state == MOVE_LOOK_MOVING){
            _look_state = MOVE_LOOK_PAUSE;
            _look_timer.start(_look_pause_time);
        }

        if((_look_angle_index >= _look_num_angs ) ||
            (_look_angle_index >= _look_max_angs )){
            _look_state = MOVE_LOOK_COMPLETE;
        }
    }

    if(_look_state == MOVE_LOOK_MOVING){
        float move_angle = 0.0;
        if(_look_angle_index == 0){
            move_angle = _look_angles[_look_angle_index];
        }
        else{
            move_angle = _look_angles[_look_angle_index] -
                            _look_angles[_look_angle_index-1];
        }
        _move_controller->turn_to_angle_ctrl_pos(move_angle);
    }
    else if(_look_state == MOVE_LOOK_PAUSE){
        _move_basic->stop();
    }
    else{
        _move_basic->stop();
    }

    return _look_state;
}

void MoveLook::force_move(){
    _look_state = MOVE_LOOK_MOVING;
    _look_angle_index++;
    _look_timer.start(_look_move_time);
    _move_controller->reset();
}

void MoveLook::reset(){
    _look_state = MOVE_LOOK_START;
    _look_angle_index = 0;
    _move_controller->reset();
}

