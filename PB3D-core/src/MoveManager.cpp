//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveManager.h"

MoveManager::MoveManager(Encoder* encoder_left,
                        Encoder* encoder_right){
    _encoder_left = encoder_left;
    _encoder_right = encoder_right;

}

//------------------------------------------------------------------------------
void MoveManager::begin(){
    _encoder_left->begin();
    _encoder_right->begin();

    _motor_shield.begin();  // create with the default frequency 1.6KHz
    _move_controller.begin(_encoder_left,_encoder_right);
    _move_basic.begin(&_move_controller);

    // Randomly generate a move type and start the timer
    _move_compound_code = EMoveCompound((0,uint8_t(_move_compound_count)));
    _move_update_time = random(_move_update_min_time,_move_update_max_time);

    _move_timer.start(_move_update_time);
    _submove_timer.start(0);
  }

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void MoveManager::update(){
    if(!enabled){return;}

    if(_move_timer.finished()){
        EMoveCompound move = EMoveCompound(
                             random(0,_move_compound_count));
        _update_compound_move(move);
    }
}

void MoveManager::update(EMoveCompound move){
    if(!enabled){return;}

    if(_move_timer.finished()){
        _update_compound_move(move);
    }
}

void MoveManager::force_update(EMoveCompound move){
    if(!enabled){return;}

    _update_compound_move(move);
}

//---------------------------------------------------------------------------
// GO
void MoveManager::go(){
    if(!enabled){return;}

    switch(_move_compound_code){
        case MOVE_C_ZIGZAG:
            zig_zag();
            break;
        case MOVE_C_SPIRAL:
            spiral();
            break;
        case MOVE_C_CIRCLE:
            circle();
            break;
        case MOVE_C_LOOK:
            look_around();
            break;
        default:
            forward();
            break;
    }
}

//---------------------------------------------------------------------------
// GET,SET and RESET functions: full implementation
float MoveManager::get_forward_speed(){
    return _move_basic.get_forward_speed();
}

float MoveManager::get_back_speed(){
    return _move_basic.get_back_speed();
}

float MoveManager::get_turn_speed(){
    return _move_basic.get_turn_speed();
}

void MoveManager::set_speed_base_multiplier(float multiplier){
    _move_basic.set_speed_base_multiplier(multiplier);
}

void MoveManager::set_speed_mood_multiplier(float multiplier){
    _move_basic.set_speed_mood_multiplier(multiplier);
}

void MoveManager::set_speed_danger_multiplier(EDangerCode danger_code){
    _move_basic.set_speed_danger_multiplier(danger_code);
}

void MoveManager::change_turn_dir(){
    if(_move_spiral.turn_direction == MOVE_TURN_LEFT){
        _move_spiral.turn_direction = MOVE_TURN_RIGHT;
    }
    else{
        _move_spiral.turn_direction = MOVE_TURN_LEFT;
    }

    if(_move_circle.turn_direction == MOVE_TURN_LEFT){
        _move_circle.turn_direction = MOVE_TURN_RIGHT;
    }
    else{
        _move_circle.turn_direction = MOVE_TURN_LEFT;
    }
}

//---------------------------------------------------------------------------
// CALCULATORS


//---------------------------------------------------------------------------
// BASIC MOVEMENT FUNCTIONS - GENERIC (switched by _move_control_code var)

void MoveManager::stop(){
    _update_basic_move(MOVE_B_STOP);
    _move_basic.stop();
}

void MoveManager::stop_no_update(){
    _move_basic.stop();
}

void MoveManager::forward(){
    _update_basic_move(MOVE_B_FORWARD);
    _move_basic.forward();
}

void MoveManager::back(){
    _update_basic_move(MOVE_B_BACK);
    _move_basic.back();
}

void MoveManager::left(){
    _update_basic_move(MOVE_B_LEFT);
    _move_basic.left();
}

void MoveManager::right(){
    _update_basic_move(MOVE_B_RIGHT);
    _move_basic.right();
}


void MoveManager::forward_left(){
    _update_basic_move(MOVE_B_FORLEFT);
    _move_basic.forward_left();
}

void MoveManager::forward_left_diff_frac(float diff_frac){
    _update_basic_move(MOVE_B_FORLEFT);
    _move_basic.forward_left_diff_frac(diff_frac);
}

void MoveManager::forward_left_diff_speed(float diff_speed){
    _update_basic_move(MOVE_B_FORLEFT);
    _move_basic.forward_left_diff_speed(diff_speed);
}


void MoveManager::forward_right(){
    _update_basic_move(MOVE_B_FORRIGHT);
    _move_basic.forward_right();
}

void MoveManager::forward_right_diff_frac(float diff_frac){
    _update_basic_move(MOVE_B_FORRIGHT);
    _move_basic.forward_right_diff_frac(diff_frac);
}

void MoveManager::forward_right_diff_speed(float diff_speed){
    _update_basic_move(MOVE_B_FORRIGHT);
    _move_basic.forward_right_diff_speed(diff_speed);
}

//============================================================================
// ENCODER/PID CONTROLLED MOVEMENT FUNCTIONS
//============================================================================

// NOTE: position can be negative to move backwards
void MoveManager::to_dist_ctrl_pos(float set_dist_left, float set_dist_right){
    _update_basic_move(MOVE_B_TODIST_CPOS);
    _move_controller.to_position(set_dist_left,set_dist_right);
}

void MoveManager::turn_to_angle_ctrl_pos(float set_angle){
    _update_basic_move(MOVE_B_TOANG_CPOS);
    _move_controller.turn_to_angle_ctrl_pos(set_angle);
}

EMoveControlState MoveManager::to_dist_ctrl_speed(float speed_left,
                                                  float speed_right,
                                                  float set_dist_left,
                                                  float set_dist_right){

    _update_basic_move(MOVE_B_TODIST_CSpeed);
    return _move_controller.to_dist_ctrl_speed(speed_left,
                                               speed_right,
                                               set_dist_left,
                                               set_dist_right);
}

EMoveControlState MoveManager::to_dist_ctrl_speed(float set_dist){
    EMoveControlState ctrl_state = MOVE_CONTROL_INCOMPLETE;

    if(set_dist < 0.0){
        ctrl_state = _move_controller.to_dist_ctrl_speed(
                                        _move_basic.get_back_speed(),
                                        _move_basic.get_back_speed(),
                                        set_dist,
                                        set_dist);
    }
    else{
        ctrl_state = _move_controller.to_dist_ctrl_speed(
                                        _move_basic.get_forward_speed(),
                                        _move_basic.get_forward_speed(),
                                        set_dist,
                                        set_dist);
    }
    return ctrl_state;
}

EMoveControlState MoveManager::turn_to_angle_ctrl_speed(float set_angle){
    float set_dist = set_angle*wheel_data.circ_per_degree;
    EMoveControlState ctrl_state = MOVE_CONTROL_INCOMPLETE;

    if(set_angle > 0.0){ // Turn left
        ctrl_state = _move_controller.to_dist_ctrl_speed(
                                        -1.0*_move_basic.get_turn_speed(),
                                        _move_basic.get_turn_speed(),
                                        -1.0*set_dist,
                                        set_dist);
    }
    else if(set_angle < 0.0){
        ctrl_state = _move_controller.to_dist_ctrl_speed(
                                        _move_basic.get_turn_speed(),
                                        -1.0*_move_basic.get_turn_speed(),
                                        -1.0*set_dist,
                                        set_dist);
    }
    else{
        ctrl_state = MOVE_CONTROL_COMPLETE;
    }
    return ctrl_state;
}

//==============================================================================
// COMPOUND MOVEMENT FUNCTIONS
//==============================================================================

void MoveManager::circle(){
    _move_circle.circle();
}

void MoveManager::forward_back(){
    _update_compound_move(MOVE_C_FORWARDBACK);
    _move_forward_back.forward_back();
}

void MoveManager::forward_back(uint16_t forward_time, uint16_t back_time){
    _update_compound_move(MOVE_C_FORWARDBACK);
    _move_forward_back.forward_back(forward_time,back_time);
}

void MoveManager::spiral(){
    _update_compound_move(MOVE_C_SPIRAL);
    _move_spiral.spiral();
}

void MoveManager::spiral(EMoveTurn turn_dir){
    _update_compound_move(MOVE_C_SPIRAL);
    _move_spiral.spiral(turn_dir);
}

void MoveManager::wiggle(){
    _update_compound_move(MOVE_C_WIGGLE);
    _move_wiggle.wiggle();
}

void MoveManager::wiggle(uint16_t left_time, uint16_t right_time){
    _update_compound_move(MOVE_C_WIGGLE);
    _move_wiggle.wiggle(left_time,right_time);
}

void MoveManager::zig_zag(){
    _update_compound_move(MOVE_C_ZIGZAG);
    _mov_zig_zag.zig_zag();
}

EMoveLookState MoveManager::look_around(){
    _update_compound_move(MOVE_C_LOOK);
    return _move_look.look_around();
}

void MoveManager::force_look_move(){
    _move_look.force_move();
}

void MoveManager::reset_look(){
    _move_look.reset();
}

//----------------------------------------------------------------------------
// Update helper functions to reset movements
void MoveManager::_update_basic_move(EMoveBasic move){
    if(_move_basic_code != move){
        _move_basic_code = move;
        _move_controller.reset();
    }
}

void MoveManager::_update_compound_move(EMoveCompound move){
    if(_move_compound_code != move || _move_timer.finished()){
        _move_compound_code = move;
        _move_update_time = random(_move_update_min_time,_move_update_max_time);

        if(_move_compound_code == MOVE_C_SPIRAL){
            _move_spiral.reset();
            _move_spiral.turn_direction = EMoveTurn(random(0,MOVE_TURN_COUNT));
            _move_update_time = _move_spiral.duration;
        }
        else if(_move_compound_code == MOVE_C_CIRCLE){
            _move_circle.turn_direction = EMoveTurn(random(0,MOVE_TURN_COUNT));
        }
        else if(_move_compound_code == MOVE_C_LOOK){
            _move_look.reset();
            _move_update_time = _move_look.get_look_total_time();
        }

        _move_timer.start(_move_update_time);
        _submove_timer.start(0);
    }
}

