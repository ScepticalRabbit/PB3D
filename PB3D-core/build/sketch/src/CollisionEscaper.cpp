#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/CollisionEscaper.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "CollisionEscaper.h"
#include "CollisionDangerCodes.h"


void CollisionEscaper::set_move_obj(MoveManager* inMove){
    _move_manager = inMove;
}


void CollisionEscaper::update_escape_decision(uint8_t check_vec[]){
    // _checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
    // NOTE: first thing in the tree is dealt with first

    // 1) BMPL: reverse a full body length and turn +90 deg away
    if(check_vec[0] == DANGER_CLOSE){
        _escape_count = ESCAPE_REV;
        _escape_dist = 1.0*_def_rev_dist;
        _escape_angle = -1.0*_def_hard_turn;
    }
    // 2) BMPR: reverse a full body length and turn -90 deg away
    else if(check_vec[1] == DANGER_CLOSE){
        _escape_count = ESCAPE_REV;
        _escape_dist = 1.0*_def_rev_dist;
        _escape_angle = 1.0*_def_hard_turn;
    }
    // 7) LSRD: check cliff-close/far, check obstacle-close/far
    else if(check_vec[6] >= DANGER_FAR){
        if(check_vec[6] == DANGER_CLOSE){
        _escape_count = ESCAPE_REV;
        _escape_dist = 1.0*_def_rev_dist;
        _escape_angle = _get_rand_turn_dir()*_def_hard_turn;
        }
        else{
        _escape_count = ESCAPE_NOREV;
        _escape_dist = 0.0;
        _escape_angle = _get_rand_turn_dir()*_def_hard_turn;
        }
    }
    // 4) LSRL: check far (norev,+45deg), check close (rev,+90deg)
    else if(check_vec[3] >= DANGER_FAR){
        if(check_vec[3] == DANGER_CLOSE){
        _escape_count = ESCAPE_REV;
        _escape_dist = 0.8*_def_rev_dist;
        _escape_angle = -1.25*_def_mod_turn;
        }
        else{
        _escape_count = ESCAPE_NOREV;
        _escape_dist = 0.0;
        _escape_angle = -1.0*_def_mod_turn;
        }

        if(check_vec[4] >= DANGER_FAR){ // If the right laser is tripped as well then we are in a corner, do a 180
        _escape_angle = -180.0 + _get_rand_turn_dir()*float(random(0,45));
        }
        else if(check_vec[2] >= DANGER_FAR){ // If the ultrasonic sensor is tripped as well then turn harder
        _escape_angle = -1.0*_def_hard_turn;
        }
    }
    // 5) LSRR: check far (norev,-45deg), check close (rev,-90deg)
    else if(check_vec[4] >= DANGER_FAR){
        if(check_vec[4] == DANGER_CLOSE){
        _escape_count = ESCAPE_REV;
        _escape_dist = 0.8*_def_rev_dist;
        _escape_angle = 1.25*_def_mod_turn;
        }
        else{
        _escape_count = ESCAPE_NOREV;
        _escape_dist = 0.0;
        _escape_angle = 1.0*_def_mod_turn;
        }

        if(check_vec[3] >= DANGER_FAR){ // If the right laser is tripped as well then we are in a corner, do a 180
        _escape_angle = 180.0 + _get_rand_turn_dir()*float(random(0,45));
        }
        else if(check_vec[2] >= DANGER_FAR){ // If the ultrasonic sensor is tripped as well then turn harder
        _escape_angle = -1.0*_def_hard_turn;
        }
    }
    // 3) US: check far (norev,+/-45deg), check close (rev,+/-90deg)
    else if(check_vec[2] >= DANGER_FAR){
        if(check_vec[2] == DANGER_CLOSE){
        _escape_count = ESCAPE_REV;
        _escape_dist = _def_rev_dist;
        _escape_angle = _get_rand_turn_dir()*_def_hard_turn;
        }
        else{
        _escape_count = ESCAPE_NOREV;
        _escape_dist = 0.0;
        _escape_angle = _get_rand_turn_dir()*_def_hard_turn;
        }
    }
    // 6) LSRU: check overhang-close/far
    else if(check_vec[5] >= DANGER_FAR){
        if(check_vec[5] == DANGER_CLOSE){
        _escape_count = ESCAPE_REV;
        _escape_dist = _def_rev_dist;
        _escape_angle = _get_rand_turn_dir()*_def_hard_turn;
        }
        else{
        _escape_count = ESCAPE_NOREV;
        _escape_dist = 0.0;
        _escape_angle = _get_rand_turn_dir()*_def_hard_turn;
        }
    }
}


void CollisionEscaper::set_escape_start(uint8_t check_vec[]){
    update_escape_decision(check_vec);  // This is the decision tree
}


void CollisionEscaper::escape(){
    if(_escape_count == 0){ // Use the first escape count to reverse by a set distance
        int8_t isComplete = _move_manager->toDistCtrlSpd(_escape_dist);
        if(isComplete > 0){ // this portion of the escape is complete
            _escape_count = 1;
        }
    }
    else if(_escape_count == 1){
        int8_t isComplete = _move_manager->turnToAngleCtrlSpd(_escape_angle);
        if(isComplete > 0){ // this portion of the escape is complete
            _escape_count = 2;
        }
    }
    else{
        _move_manager->stop();
    }
}


bool CollisionEscaper::get_escape_flag(){
    bool outFlag = false;
    if(_escape_count <= 1){
        outFlag = true;
    }
    return outFlag;
}


int8_t CollisionEscaper::get_escape_turn(uint8_t check_vec[]){
    //_update_check_vec();    // Check all collision sensors - used for decision tree
    update_escape_decision(check_vec);  // This is the decision tree
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


