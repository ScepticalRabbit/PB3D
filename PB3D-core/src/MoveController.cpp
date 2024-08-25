//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveController.h"


MoveController::MoveController(Adafruit_MotorShield* motor_shield){
    _motor_shield = motor_shield;
}

void MoveController::begin(Encoder* encoder_left, Encoder* encoder_right){
    _motor_right = _motor_shield->getMotor(MOTOR_RIGHT);
    _motor_left = _motor_shield->getMotor(MOTOR_LEFT);
    _encoder_left = encoder_left;
    _encoder_right = encoder_right;

    // Start the speed PIDs
    _speed_PID_left.begin();
    _speed_PID_right.begin();
    _speed_PID_left.set_output_limits(_speed_PID_min_power, 255.0);
    _speed_PID_right.set_output_limits(_speed_PID_min_power, 255.0);
    _speed_PID_left.set_sample_time(_encoder_left->get_speed_update_time());
    _speed_PID_right.set_sample_time(_encoder_right->get_speed_update_time());
    // Start the position PIDs
    _pos_PID_left.begin();
    _pos_PID_right.begin();
    _pos_PID_left.set_output_limits(-1.0*_pos_PID_max_speed,_pos_PID_max_speed);
    _pos_PID_right.set_output_limits(-1.0*_pos_PID_max_speed,_pos_PID_max_speed);
    _pos_PID_left.set_sample_time(_encoder_left->get_speed_update_time()*2);
    _pos_PID_right.set_sample_time(_encoder_right->get_speed_update_time()*2);
}

void MoveController::reset(){
    // Reset Position Control PIDs and Variables
    _set_point_rel_counts_left = 0;
    _set_point_rel_counts_right = 0;
    _pos_PID_left.set_set_point(0.0);
    _pos_PID_right.set_set_point(0.0);
    _pos_PID_left.set_output(0.0);
    _pos_PID_right.set_output(0.0);
    _pos_PID_left.set_controller_on(PID_OFF);
    _pos_PID_right.set_controller_on(PID_OFF);
    _pos_at_left = false;
    _pos_at_right = false;
    _pos_at_both = false;
    // Reset Speed PIDs
    _speed_PID_left.set_set_point(0.0);
    _speed_PID_right.set_set_point(0.0);
    _speed_PID_left.set_output(0.0);
    _speed_PID_right.set_output(0.0);
    _speed_PID_left.set_controller_on(PID_OFF);
    _speed_PID_right.set_controller_on(PID_OFF);
}

void MoveController::at_speed(float speed_left,float speed_right){
    // Check if the left/right PIDs are on if not turn them on
    if(!_speed_PID_left.get_controller_on()){
        _speed_PID_left.set_controller_on(PID_ON);
        _speed_PID_left.set_Pgain_only(_speed_P_rev);
        if(speed_left < 0.0){
        _speed_PID_left.set_controller_dir(PID_REVERSE);

        }
        else{
        _speed_PID_left.set_controller_dir(PID_DIRECT);
        }
        if((speed_left < 0.0) && (speed_right < 0.0)){
        _speed_PID_right.set_Pgain_only(_speed_P_rev);
        _speed_PID_left.set_Pgain_only(_speed_P_rev);
        }
    }
    if(!_speed_PID_right.get_controller_on()){
        _speed_PID_right.set_controller_on(PID_ON);
        _speed_PID_right.set_Pgain_only(_speed_P_rev);
        if(speed_right < 0.0){
        _speed_PID_right.set_controller_dir(PID_REVERSE);
        }
        else{
        _speed_PID_right.set_controller_dir(PID_DIRECT);
        }
        if((speed_left < 0.0) && (speed_right < 0.0)){
        _speed_PID_right.set_Pgain_only(_speed_P_rev);
        _speed_PID_left.set_Pgain_only(_speed_P_rev);
        }
    }

    // Update the set point
    _speed_PID_left.set_set_point(speed_left);
    _speed_PID_right.set_set_point(speed_right);

    // Update left and right speed PIDs
    _speed_PID_left.update(_encoder_left->get_smooth_speed_mmps());
    _speed_PID_right.update(_encoder_right->get_smooth_speed_mmps());

    // If the speed is negative then set motors to run backward
    if(speed_left < 0.0){
        _motor_left->run(BACKWARD);
    }
    else{
        _motor_left->run(FORWARD);
    }
    if(speed_right < 0.0){
        _motor_right->run(BACKWARD);
    }
    else{
        _motor_right->run(FORWARD);
    }
    _motor_left->setSpeed(int(_speed_PID_left.get_output()));
    _motor_right->setSpeed(int(_speed_PID_right.get_output()));
}

//----------------------------------------------------------------------------
void MoveController::to_position(float set_pos_left, float set_pos_right){
    // LEFT
    if(!_pos_PID_left.get_controller_on()){
        _pos_PID_left.set_controller_on(PID_ON);
        _speed_PID_left.set_controller_on(PID_ON);
        //_speed_PID_left.set_PID_gains(_speed_P,_speed_I,_speed_D);
    }
    // RIGHT
    if(!_pos_PID_right.get_controller_on()){
        _pos_PID_right.set_controller_on(PID_ON);
        _speed_PID_right.set_controller_on(PID_ON);
        //_speed_PID_right.set_PID_gains(_speed_P,_speed_I,_speed_D);
    }

    // Check if the set point passed to the function has changed
    // LEFT
    int32_t checkSetPointL =  round(set_pos_left/_encoder_left->get_mm_per_count());
    if(checkSetPointL != _set_point_rel_counts_left){
        _set_point_rel_counts_left = checkSetPointL;
        _start_encoder_count_left = _encoder_left->get_count();
        _pos_PID_left.set_set_point(float(_set_point_rel_counts_left));
    }
    // RIGHT
    int32_t checkSetPointR =  round(set_pos_right/_encoder_right->get_mm_per_count());
    if(checkSetPointR != _set_point_rel_counts_right){
        _set_point_rel_counts_right = checkSetPointR;
        _start_encoder_count_right = _encoder_right->get_count();
        _pos_PID_right.set_set_point(float(_set_point_rel_counts_right));
    }

    // Update the relative count and send it to the PIDs
    // LEFT
    _curr_relative_count_left = _encoder_left->get_count()-_start_encoder_count_left;
    _pos_PID_left.update(_curr_relative_count_left);
    // RIGHT
    _curr_relative_count_right = _encoder_right->get_count()-_start_encoder_count_right;
    _pos_PID_right.update(_curr_relative_count_right);

    // Update the speed PIDs
    // LEFT
    _speed_PID_left.update(_encoder_left->get_smooth_speed_mmps());
    // RIGHT
    _speed_PID_right.update(_encoder_right->get_smooth_speed_mmps());

    // Check that the PID is sending a signal above the min speed
    // LEFT
    if(round(abs(_pos_PID_left.get_output())) < _pos_PID_min_speed){
        if(_pos_PID_left.get_output() < 0.0){
        _pos_PID_left.set_output(-1.0*_pos_PID_min_speed);
        }
        else{
        _pos_PID_left.set_output(_pos_PID_min_speed);
        }
    }
    // RIGHT
    if(round(abs(_pos_PID_right.get_output())) < _pos_PID_min_speed){
        if(_pos_PID_right.get_output() < 0.0){
        _pos_PID_right.set_output(-1.0*_pos_PID_min_speed);
        }
        else{
        _pos_PID_right.set_output(_pos_PID_min_speed);
        }
    }

    // Move forward or back based on the PID value
    // LEFT
    if(_curr_relative_count_left < (_set_point_rel_counts_left-_pos_tol)){
        _speed_PID_left.set_set_point(_pos_PID_left.get_output());
        _speed_PID_left.set_controller_dir(PID_DIRECT);
        _motor_left->run(FORWARD);
        _motor_left->setSpeed(int(_speed_PID_left.get_output()));
    }
    else if(_curr_relative_count_left > (_set_point_rel_counts_left+_pos_tol)){
        _speed_PID_left.set_set_point(_pos_PID_left.get_output());
        _speed_PID_left.set_controller_dir(PID_REVERSE);
        _motor_left->run(BACKWARD);
        _motor_left->setSpeed(int(_speed_PID_left.get_output()));
    }
    else{
        _pos_PID_left.set_output(0.0);
        _speed_PID_left.set_output(0.0);
        _speed_PID_left.set_set_point(0.0);
        _motor_left->run(RELEASE);
        _motor_left->setSpeed(0);
        _pos_at_left = true;
    }
    // RIGHT
    if(_curr_relative_count_right < (_set_point_rel_counts_right-_pos_tol)){
        _speed_PID_right.set_set_point(_pos_PID_right.get_output());
        _speed_PID_right.set_controller_dir(PID_DIRECT);
        _motor_right->run(FORWARD);
        _motor_right->setSpeed(int(_speed_PID_right.get_output()));
    }
    else if(_curr_relative_count_right > (_set_point_rel_counts_right+_pos_tol)){
        _speed_PID_right.set_set_point(_pos_PID_right.get_output());
        _speed_PID_right.set_controller_dir(PID_REVERSE);
        _motor_right->run(BACKWARD);
        _motor_right->setSpeed(int(_speed_PID_right.get_output()));
    }
    else{
        _pos_PID_right.set_output(0.0);
        _speed_PID_right.set_output(0.0);
        _speed_PID_right.set_set_point(0.0);
        _motor_right->run(RELEASE);
        _motor_right->setSpeed(0);
        _pos_at_right = true;
    }

    if(_pos_at_left && _pos_at_right){
        _pos_at_both = true;
    }
}