//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveBasic.h"

void MoveBasic::begin(MoveController* move_controller){
    _move_controller = move_controller;

    // M1 is the right motor, M2 is the left motor
    _motor_right = _motor_shield->getMotor(MOTOR_RIGHT);
    _motor_left = _motor_shield->getMotor(MOTOR_LEFT);

    // Set the speed to start, from 0 (off) to 255 (max  speed)
    _motor_right->setSpeed(_def_forward_power);
    _motor_right->run(FORWARD);
    _motor_right->run(RELEASE);
    _motor_left->setSpeed(_def_forward_power);
    _motor_left->run(FORWARD);
    _motor_left->run(RELEASE);
}

//----------------------------------------------------------------------------
// Move Stop - same regardless of control mode
void MoveBasic::stop(){
    _motor_left->run(RELEASE);
    _motor_right->run(RELEASE);
    _motor_left->setSpeed(0);
    _motor_right->setSpeed(0);
}

//----------------------------------------------------------------------------
// Move Forward
void MoveBasic::forward(){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        forward_speed(_cur_forward_speed);
    }
    else{
        forward_power(_cur_forward_power);
    }
}


//----------------------------------------------------------------------------
// Move Back
void MoveBasic::back(){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        back_speed(_cur_back_speed);
    }
    else{
        back_power(_cur_back_power);
    }
}

//----------------------------------------------------------------------------
// Move Left
void MoveBasic::left(){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        left_speed(_cur_turn_speed);
    }
    else{
        left_power(_cur_turn_power);
    }
}


//----------------------------------------------------------------------------
// Move Right
void MoveBasic::right(){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        right_speed(_cur_turn_speed);
    }
    else{
        right_power(_cur_turn_power);
    }
}

//----------------------------------------------------------------------------
// Move Forward Left
void MoveBasic::forward_left(){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        forward_left(_cur_forward_speed, _curTurnSpeedDiff);
    }
    else{
        forward_left(_cur_forward_power, _cur_turn_power_diff);
    }
}

void MoveBasic::forward_left_diff_frac(float diffFrac){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        float speedDiff = _cur_forward_speed*diffFrac;
        forward_left(_cur_forward_speed, speedDiff);
    }
    else{
        uint8_t PowerDiff = round(diffFrac*float(_cur_forward_power));
        forward_left(_cur_forward_power, PowerDiff);
    }
}

void MoveBasic::forward_left(float inSpeedDiff){
    forward_left_speed(_cur_turn_speed,inSpeedDiff);
}
void MoveBasic::forward_left(float inSpeed, float inSpeedDiff){
    forward_left_speed(inSpeed,inSpeedDiff);
}

void MoveBasic::forward_left(uint8_t inPowerDiff){
    forward_left_power(_cur_turn_power,inPowerDiff);
}
void MoveBasic::forward_left(uint8_t inPower, uint8_t inPowerDiff){
    forward_left_power(inPower,inPowerDiff);
}

//----------------------------------------------------------------------------
// Move Forward Right
void MoveBasic::forward_right(){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        forward_right(_cur_forward_speed, _curTurnSpeedDiff);
    }
    else{
        forward_right(_cur_forward_power, _cur_turn_power_diff);
    }
}

void MoveBasic::forward_right_diff_frac(float diffFrac){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        float speedDiff = _cur_forward_speed*diffFrac;
        forward_right(_cur_forward_speed, speedDiff);
    }
    else{
        uint8_t PowerDiff = round(diffFrac*float(_cur_forward_power));
        forward_right(_cur_forward_power, PowerDiff);
    }
}

void MoveBasic::forward_right(float inSpeedDiff){
    forward_right_speed(_cur_turn_speed,inSpeedDiff);
}
void MoveBasic::forward_right(float inSpeed, float inSpeedDiff){
    forward_right_speed(inSpeed,inSpeedDiff);
}

void MoveBasic::forward_right(uint8_t inPowerDiff){
    forward_right_power(_cur_turn_power,inPowerDiff);
}
void MoveBasic::forward_right(uint8_t inPower, uint8_t inPowerDiff){
    forward_right_power(inPower,inPowerDiff);
}

//============================================================================
// BASIC MOVEMENT FUNCTIONS - CONTROL BY POWER
void MoveBasic::forward_power(uint8_t inPower){
    _motor_left->run(FORWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveBasic::back_power(uint8_t inPower){
    _motor_left->run(BACKWARD);
    _motor_right->run(BACKWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveBasic::left_power(uint8_t inPower){
    _motor_left->run(BACKWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveBasic::right_power(uint8_t inPower){
    _motor_left->run(FORWARD);
    _motor_right->run(BACKWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveBasic::forward_left_power(uint8_t inPower, uint8_t inPowerDiff){

    _motor_left->run(FORWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower-inPowerDiff/2);
    _motor_right->setSpeed(inPower+inPowerDiff/2);
}

void MoveBasic::forward_right_power(uint8_t inPower, uint8_t inPowerDiff){
    _motor_left->run(FORWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower+inPowerDiff/2);
    _motor_right->setSpeed(inPower-inPowerDiff/2);
}

//============================================================================
// BASIC MOVEMENT - SPEED CONTROL with PID
void MoveBasic::forward_speed(float inSpeed){
    _move_controller->at_speed(inSpeed,inSpeed);
}

void MoveBasic::back_speed(float inSpeed){
    _move_controller->at_speed(inSpeed,inSpeed);
}

void MoveBasic::left_speed(float inSpeed){
    _move_controller->at_speed(-1.0*inSpeed,inSpeed);
}

void MoveBasic::forward_left_speed(float inSpeed, float inSpeedDiff){
    _move_controller->at_speed(inSpeed-(0.5*inSpeedDiff),inSpeed+(0.5*inSpeedDiff));
}

void MoveBasic::right_speed(float inSpeed){
    _move_controller->at_speed(inSpeed,-1.0*inSpeed);
}

void MoveBasic::forward_right_speed(float inSpeed, float inSpeedDiff){
    _move_controller->at_speed(inSpeed+(0.5*inSpeedDiff),inSpeed-(0.5*inSpeedDiff));
}

/*
// TODO fix below

//------------------------------------------------------------------------------
// NOTE: position can be negative to move backwards
void MoveBasic::to_dist_ctrl_pos(float setDist){
    _update_basic_move(MOVE_B_TODIST_CPOS);
    to_position(setDist,setDist);
}
void MoveBasic::to_dist_ctrl_pos(float setDistL, float setDistR){
    _update_basic_move(MOVE_B_TODIST_CPOS);
    to_position(setDistL,setDistR);
}

void MoveBasic::turn_to_angle_ctrl_pos(float setAngle){
    _update_basic_move(MOVE_B_TOANG_CPOS);
    float arcLeng = setAngle*_wheel_circ_ang;
    to_position(-1.0*arcLeng,arcLeng);
}

int8_t MoveBasic::to_dist_ctrl_speed(float speedL, float speedR,
    float setDistL, float setDistR){
    int8_t isComplete = 0;
    _update_basic_move(MOVE_B_TODIST_CSpeed);

    // If the set distance changes outside tolerance force update
    if(!((setDistL >= (_to_dist_set_pt_left-_to_dist_tol)) && (setDistL <= (_to_dist_set_pt_left+_to_dist_tol)))){
        _to_dist_set_pt_left = setDistL;
        _update_basic_move(MOVE_B_FORCEUPD);
    }
    if(!((setDistR >= (_to_dist_set_pt_right-_to_dist_tol)) && (setDistR <= (_to_dist_set_pt_right+_to_dist_tol)))){
        _to_dist_set_pt_right = setDistR;
        _update_basic_move(MOVE_B_FORCEUPD);
    }

    // At the start we store our target counts for each encode
    if(_encoder_count_start){
        uint16_t timeoutL = calc_timeout(speedL,setDistL);
        uint16_t timeoutR = calc_timeout(speedR,setDistR);
        if(timeoutL > timeoutR){
        _timeout_timer.start(timeoutL);
        }
        else{
        _timeout_timer.start(timeoutR);
        }

        _encoder_count_start = false;
        _encoder_count_diff_left = int32_t(setDistL/_encoder_left->get_mm_per_count());
        _enc_count_diff_right = int32_t(setDistR/_encoder_right->get_mm_per_count());
        _end_encoder_count_left = _start_encoder_count_left + _encoder_count_diff_left;
        _end_encoder_count_right = _start_encoder_count_right + _enc_count_diff_right;


        Serial.print("MMPCount= "); Serial.print(_encoder_left->get_mm_per_count());
        Serial.print(",SetDistL= "); Serial.print(setDistL); Serial.print(",SetDistR= "); Serial.print(setDistR);
        Serial.print(",ECDiffL= "); Serial.print(_encoder_count_diff_left); Serial.print(",ECDiffR= "); Serial.print(_enc_count_diff_right);
        Serial.println();
        Serial.print("StartECount: L="); Serial.print(_start_encoder_count_left); Serial.print(", R="); Serial.print(_start_encoder_count_right);
        Serial.println();
        Serial.print("EndECount: L="); Serial.print(_end_encoder_count_left); Serial.print(", R="); Serial.print(_end_encoder_count_right);
        Serial.println();
        Serial.println();
    }

    if(_timeout_timer.finished()){
        isComplete = 2;
    }
    else{
        if((setDistL > 0.0) && (setDistR > 0.0)){ // Go forward
        if((_encoder_left->get_count() <= _end_encoder_count_left)||(_encoder_right->get_count() <= _end_encoder_count_right)){
            at_speed(abs(speedL),abs(speedR));
        }
        else{
            isComplete = 1;
            stop_no_update();
        }
        }
        else if((setDistL < 0.0) && (setDistR > 0.0)){ // Turn left
            if((_encoder_left->get_count() >= _end_encoder_count_left)||(_encoder_right->get_count() <= _end_encoder_count_right)){
                at_speed(-1.0*abs(speedL),abs(speedR));
            }
            else{
                isComplete = 1;
                stop_no_update();
            }
        }
        else if((setDistL > 0.0) && (setDistR < 0.0)){
            if((_encoder_left->get_count() <= _end_encoder_count_left)||(_encoder_right->get_count() >= _end_encoder_count_right)){
                at_speed(abs(speedL),-1.0*abs(speedR));
            }
            else{
                isComplete = 1;
                stop_no_update();
            }
        }
        else if((setDistL < 0.0) && (setDistR < 0.0)){ // Turn right
            if((_encoder_left->get_count() >= _end_encoder_count_left)||(_encoder_right->get_count() >= _end_encoder_count_right)){
                at_speed(-1.0*abs(speedL),-1.0*abs(speedR));
            }
            else{
                isComplete = 1;
                stop_no_update();
            }
        }
        else{
            isComplete = 1;
            stop_no_update();
        }
    }
    return isComplete;
}

int8_t MoveBasic::to_dist_ctrl_speed(float setDist){
    int8_t isComplete = 0;
    if(setDist < 0.0){
        isComplete = to_dist_ctrl_speed(_cur_back_speed,_cur_back_speed,setDist,setDist);
    }
    else{
        isComplete = to_dist_ctrl_speed(_cur_forward_speed,_cur_forward_speed,setDist,setDist);
    }
    return isComplete;
}

int8_t MoveBasic::turn_to_angle_ctrl_speed(float setAngle){
    float setDist = setAngle*_wheel_circ_ang;
    int8_t isComplete = 0;
    if(setAngle > 0.0){ // Turn left
        isComplete = to_dist_ctrl_speed(-1.0*_cur_turn_speed,_cur_turn_speed,-1.0*setDist,setDist);
    }
    else if(setAngle < 0.0){
        isComplete = to_dist_ctrl_speed(_cur_turn_speed,-1.0*_cur_turn_speed,-1.0*setDist,setDist);
    }
    else{
        isComplete = 1;
    }
    return isComplete;
}
*/
