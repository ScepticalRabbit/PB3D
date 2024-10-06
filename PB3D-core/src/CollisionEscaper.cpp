//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include <PB3DConstants.h>
#include "CollisionEscaper.h"


void CollisionEscaper::set_move_obj(MoveManager* inMove){
    _move_manager = inMove;
}


//==============================================================================
//TODO: update this to work with bumpers and new lasers
void CollisionEscaper::update_escape_decision(uint8_t check_bumpers[],
                                              uint8_t check_lasers[]){

    // BUMPERS
    if(check_bumpers[BUMP_LEFT] == DANGER_CLOSE){
        _escape_state = ESC_STATE_REV;
        _escape_dist = _def_rev_dist;
        _escape_angle = _hard_turn_right;
    }
    else if(check_bumpers[BUMP_RIGHT] == DANGER_CLOSE){
        _escape_state = ESC_STATE_REV;
        _escape_dist = _def_rev_dist;
        _escape_angle = _hard_turn_left;
    }
    else if(check_bumpers[BUMP_BACK] == DANGER_CLOSE){
        _escape_state = ESC_STATE_REV;
        _escape_dist = -1.0*_def_rev_dist;
        _escape_angle = 0.0;
    }
    // LASERS: CLIFF EDGES
    else if(check_lasers[LASER_DOWN_LEFT] >= DANGER_FAR){
        if(check_lasers[6] == DANGER_CLOSE){
            _escape_state = ESC_STATE_REV;
            _escape_dist = _def_rev_dist;
            _escape_angle = _get_rand_turn_dir()*_hard_turn_left;
        }
        else{
            _escape_state = ESC_STATE_TURN;
            _escape_dist = 0.0;
            _escape_angle = _get_rand_turn_dir()*_hard_turn_left;
        }
    }

    // 4) LSRL: check far (norev,+45deg), check close (rev,+90deg)
    else if(check_lasers[3] >= DANGER_FAR){
        if(check_lasers[3] == DANGER_CLOSE){
            _escape_state = ESC_STATE_REV;
            _escape_dist = 0.8*_def_rev_dist;
            _escape_angle = -1.25*_mod_turn;
        }
        else{
            _escape_state = ESC_STATE_TURN;
            _escape_dist = 0.0;
            _escape_angle = -1.0*_mod_turn;
        }

        if(check_lasers[4] >= DANGER_FAR){ // If the right laser is tripped as well then we are in a corner, do a 180
            _escape_angle = -180.0 + _get_rand_turn_dir()*float(random(0,45));
        }
        else if(check_lasers[2] >= DANGER_FAR){ // If the ultrasonic sensor is tripped as well then turn harder
            _escape_angle = -1.0*_hard_turn;
        }
    }
    else if(check_lasers[4] >= DANGER_FAR){
        if(check_lasers[4] == DANGER_CLOSE){
            _escape_state = ESC_STATE_REV;
            _escape_dist = 0.8*_def_rev_dist;
            _escape_angle = 1.25*_mod_turn;
        }
        else{
            _escape_state = ESC_STATE_TURN;
            _escape_dist = 0.0;
            _escape_angle = 1.0*_mod_turn;
        }

        if(check_lasers[3] >= DANGER_FAR){ // If the right laser is tripped as well then we are in a corner, do a 180
            _escape_angle = 180.0 + _get_rand_turn_dir()*float(random(0,45));
        }
        else if(check_lasers[2] >= DANGER_FAR){ // If the ultrasonic sensor is tripped as well then turn harder
            _escape_angle = -1.0*_hard_turn;
        }
    }
    // 3) US: check far (norev,+/-45deg), check close (rev,+/-90deg)
    else if(check_lasers[2] >= DANGER_FAR){
        if(check_lasers[2] == DANGER_CLOSE){
            _escape_state = ESC_STATE_REV;
            _escape_dist = _def_rev_dist;
            _escape_angle = _get_rand_turn_dir()*_hard_turn;
        }
        else{
            _escape_state = ESC_STATE_TURN;
            _escape_dist = 0.0;
            _escape_angle = _get_rand_turn_dir()*_hard_turn;
        }
    }
    // 6) LSRU: check overhang-close/far
    else if(check_lasers[5] >= DANGER_FAR){
        if(check_lasers[5] == DANGER_CLOSE){
            _escape_state = ESC_STATE_REV;
            _escape_dist = _def_rev_dist;
            _escape_angle = _get_rand_turn_dir()*_hard_turn;
        }
        else{
            _escape_state = ESC_STATE_TURN;
            _escape_dist = 0.0;
            _escape_angle = _get_rand_turn_dir()*_hard_turn;
        }
    }
}
//==============================================================================


void CollisionEscaper::escape(){
    if(_escape_state == ESC_STATE_REV){
        int8_t is_complete = _move_manager->to_dist_ctrl_speed(_escape_dist);
        if(is_complete > 0){
            _escape_state = ESC_STATE_TURN;
        }
    }
    else if(_escape_state == ESC_STATE_TURN){
        int8_t is_complete = _move_manager->turn_to_angle_ctrl_speed(_escape_angle);
        if(is_complete > 0){
            _escape_state = ESC_STATE_COMPLETE;
        }
    }
    else{
        _move_manager->stop();
    }
}


bool CollisionEscaper::get_escape_state(){
    bool outFlag = false;
    if(_escape_state <= 1){
        outFlag = true;
    }
    return outFlag;
}


EMoveBasic CollisionEscaper::get_escape_turn(){
    if(_escape_angle > 0.0){
        return MOVE_B_LEFT;
    }
    else{
        return MOVE_B_RIGHT;
    }
}


float CollisionEscaper::_get_rand_turn_dir(){
    uint8_t randVal = random(0,2);
    float sendVal = 1.0;
    if(randVal == 1){
        sendVal = 1.0;
    }
    else{
        sendVal = -1.0;
    }
    return sendVal;
}


