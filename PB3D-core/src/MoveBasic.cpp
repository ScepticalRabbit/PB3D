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
        forward_left_speed(_cur_forward_speed, _curTurnSpeedDiff);
    }
    else{
        forward_left_power(_cur_forward_power, _cur_turn_power_diff);
    }
}

void MoveBasic::forward_left_diff_frac(float diff_fraction){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        float speedDiff = _cur_forward_speed*diff_fraction;
        forward_left_speed(_cur_forward_speed, speedDiff);
    }
    else{
        uint8_t power_diff = round(diff_fraction*float(_cur_forward_power));
        forward_left_power(_cur_forward_power, power_diff);
    }
}

void MoveBasic::forward_left_diff_speed(float diff_speed){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        forward_left_speed(_cur_forward_speed, diff_speed);
    }
    else{
        // TODO: need to calculate power for speed using calculator
        uint8_t power_diff = uint8_t(diff_speed);
        forward_left_power(_cur_forward_power, power_diff);
    }
}

//----------------------------------------------------------------------------
// Move Forward Right
void MoveBasic::forward_right(){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        forward_right_speed(_cur_forward_speed, _curTurnSpeedDiff);
    }
    else{
        forward_right_power(_cur_forward_power, _cur_turn_power_diff);
    }
}

void MoveBasic::forward_right_diff_frac(float diff_fraction){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        float speedDiff = _cur_forward_speed*diff_fraction;
        forward_right_speed(_cur_forward_speed, speedDiff);
    }
    else{
        uint8_t power_diff = round(diff_fraction*float(_cur_forward_power));
        forward_right_power(_cur_forward_power, power_diff);
    }
}

void MoveBasic::forward_right_diff_speed(float diff_speed){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        forward_right_speed(_cur_forward_speed, diff_speed);
    }
    else{
        // TODO: need to calculate power for speed using calculator
        uint8_t power_diff = uint8_t(diff_speed);
        forward_right_power(_cur_forward_power, power_diff);
    }
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

