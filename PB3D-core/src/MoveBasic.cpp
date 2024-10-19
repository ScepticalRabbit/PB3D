//==============================================================================
// PB3D: A 3D printed pet robot
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
    _motor_right->setSpeed(default_forward_power);
    _motor_right->run(FORWARD);
    _motor_right->run(RELEASE);
    _motor_left->setSpeed(default_forward_power);
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
    if(_move_control_code == MOVE_MODE_SPEED){
        forward_at_speed(_forward_speed);
    }
    else{
        forward_at_power(forward_power);
    }
}


//----------------------------------------------------------------------------
// Move Back
void MoveBasic::back(){
    if(_move_control_code == MOVE_MODE_SPEED){
        back_at_speed(_back_speed);
    }
    else{
        back_at_power(back_power);
    }
}

//----------------------------------------------------------------------------
// Move Left
void MoveBasic::left(){
    if(_move_control_code == MOVE_MODE_SPEED){
        left_at_speed(_turn_speed);
    }
    else{
        left_at_power(turn_power);
    }
}


//----------------------------------------------------------------------------
// Move Right
void MoveBasic::right(){
    if(_move_control_code == MOVE_MODE_SPEED){
        right_at_speed(_turn_speed);
    }
    else{
        right_at_power(turn_power);
    }
}

//----------------------------------------------------------------------------
// Move Forward Left
void MoveBasic::forward_left(){
    if(_move_control_code == MOVE_MODE_SPEED){
        forward_left_at_speed(_forward_speed, _turn_speed_diff);
    }
    else{
        forward_left_at_power(forward_power, turn_power_diff);
    }
}

void MoveBasic::forward_left_diff_frac(float diff_fraction){
    if(_move_control_code == MOVE_MODE_SPEED){
        float speedDiff = _forward_speed*diff_fraction;
        forward_left_at_speed(_forward_speed, speedDiff);
    }
    else{
        uint8_t power_diff = round(diff_fraction*float(forward_power));
        forward_left_at_power(forward_power, power_diff);
    }
}

void MoveBasic::forward_left_diff_speed(float diff_speed){
    if(_move_control_code == MOVE_MODE_SPEED){
        forward_left_at_speed(_forward_speed, diff_speed);
    }
    else{
        // TODO: need to calculate power for speed using calculator
        uint8_t power_diff = uint8_t(diff_speed);
        forward_left_at_power(forward_power, power_diff);
    }
}

//----------------------------------------------------------------------------
// Move Forward Right
void MoveBasic::forward_right(){
    if(_move_control_code == MOVE_MODE_SPEED){
        forward_right_at_speed(_forward_speed, _turn_speed_diff);
    }
    else{
        forward_right_at_power(forward_power, turn_power_diff);
    }
}

void MoveBasic::forward_right_diff_frac(float diff_fraction){
    if(_move_control_code == MOVE_MODE_SPEED){
        float speedDiff = _forward_speed*diff_fraction;
        forward_right_at_speed(_forward_speed, speedDiff);
    }
    else{
        uint8_t power_diff = round(diff_fraction*float(forward_power));
        forward_right_at_power(forward_power, power_diff);
    }
}

void MoveBasic::forward_right_diff_speed(float diff_speed){
    if(_move_control_code == MOVE_MODE_SPEED){
        forward_right_at_speed(_forward_speed, diff_speed);
    }
    else{
        // TODO: need to calculate power for speed using calculator
        uint8_t power_diff = uint8_t(diff_speed);
        forward_right_at_power(forward_power, power_diff);
    }
}

//============================================================================
// BASIC MOVEMENT FUNCTIONS - CONTROL BY POWER
void MoveBasic::forward_at_power(uint8_t inPower){
    _motor_left->run(FORWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveBasic::back_at_power(uint8_t inPower){
    _motor_left->run(BACKWARD);
    _motor_right->run(BACKWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveBasic::left_at_power(uint8_t inPower){
    _motor_left->run(BACKWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveBasic::right_at_power(uint8_t inPower){
    _motor_left->run(FORWARD);
    _motor_right->run(BACKWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveBasic::forward_left_at_power(uint8_t inPower, uint8_t inPowerDiff){

    _motor_left->run(FORWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower-inPowerDiff/2);
    _motor_right->setSpeed(inPower+inPowerDiff/2);
}

void MoveBasic::forward_right_at_power(uint8_t inPower, uint8_t inPowerDiff){
    _motor_left->run(FORWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower+inPowerDiff/2);
    _motor_right->setSpeed(inPower-inPowerDiff/2);
}

//============================================================================
// BASIC MOVEMENT - SPEED CONTROL with PID
void MoveBasic::forward_at_speed(float inSpeed){
    _move_controller->at_speed(inSpeed,inSpeed);
}

void MoveBasic::back_at_speed(float inSpeed){
    _move_controller->at_speed(inSpeed,inSpeed);
}

void MoveBasic::left_at_speed(float inSpeed){
    _move_controller->at_speed(-1.0*inSpeed,inSpeed);
}

void MoveBasic::forward_left_at_speed(float inSpeed, float inSpeedDiff){
    _move_controller->at_speed(inSpeed-(0.5*inSpeedDiff),inSpeed+(0.5*inSpeedDiff));
}

void MoveBasic::right_at_speed(float inSpeed){
    _move_controller->at_speed(inSpeed,-1.0*inSpeed);
}

void MoveBasic::forward_right_at_speed(float inSpeed, float inSpeedDiff){
    _move_controller->at_speed(inSpeed+(0.5*inSpeedDiff),inSpeed-(0.5*inSpeedDiff));
}

//------------------------------------------------------------------------------
void MoveBasic::set_forward_speed(float speed){
    _forward_speed = constrain(fabs(speed),min_speed,max_speed);
}

void MoveBasic::set_back_speed(float speed){
    _back_speed = -1.0*constrain(fabs(speed),min_speed,max_speed);
}

void MoveBasic::set_turn_speed(float speed){
    _turn_speed = constrain(fabs(speed),min_speed,max_speed);
}

void MoveBasic::set_speed_base_multiplier(float multiplier){
    _speed_base_multiplier = multiplier;
    _update_speed_with_multipliers();
}

void MoveBasic::set_speed_mood_multiplier(float multiplier){
    _speed_mood_multiplier = multiplier;
    _update_speed_with_multipliers();
}

void MoveBasic::set_speed_danger_multiplier(EDangerCode danger_code){
    switch(danger_code){
        case DANGER_CLOSE:
            _speed_danger_multiplier = speed_danger_true;
            break;
        default:
            _speed_danger_multiplier = speed_danger_false;
            break;
    }
    _update_speed_with_multipliers();
}

//------------------------------------------------------------------------------
void MoveBasic::_update_speed_with_multipliers(){
    float total_multiplier = _speed_base_multiplier*
                             _speed_mood_multiplier*
                             _speed_danger_multiplier;

    _forward_speed = constrain(fabs(default_forward_speed*total_multiplier),
                                    min_speed,
                                    max_speed);

    _back_speed = -1.0*constrain(fabs(default_back_speed*total_multiplier),
                                      min_speed,
                                      max_speed);

    _turn_speed = constrain(fabs(default_turn_speed*total_multiplier),
                                 min_speed,
                                 max_speed);
}

