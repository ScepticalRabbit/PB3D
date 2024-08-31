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
    _set_point_rel_counts_left = 0;
    _set_point_rel_counts_right = 0;

    _pos_PID_left.set_set_point(0.0);
    _pos_PID_right.set_set_point(0.0);
    _pos_PID_left.set_output(0.0);
    _pos_PID_right.set_output(0.0);
    _pos_PID_left.set_controller_on(PID_OFF);
    _pos_PID_right.set_controller_on(PID_OFF);
    _pos_ctrl_state = MOVE_CONTROL_INCOMPLETE;

    _speed_PID_left.set_set_point(0.0);
    _speed_PID_right.set_set_point(0.0);
    _speed_PID_left.set_output(0.0);
    _speed_PID_right.set_output(0.0);
    _speed_PID_left.set_controller_on(PID_OFF);
    _speed_PID_right.set_controller_on(PID_OFF);
}

EMoveControlState MoveController::at_speed(float set_speed_left,
                                           float set_speed_right){

    EMoveControlState ctrl_state = MOVE_CONTROL_INCOMPLETE;

    // Check if the left/right PIDs are on if not turn them on
    if(!_speed_PID_left.get_controller_on()){
        _speed_PID_left.set_controller_on(PID_ON);
        _speed_PID_left.set_Pgain_only(_speed_P_rev);
        if(set_speed_left < 0.0){
            _speed_PID_left.set_controller_dir(PID_REVERSE);

        }
        else{
            _speed_PID_left.set_controller_dir(PID_DIRECT);
        }
        if((set_speed_left < 0.0) && (set_speed_right < 0.0)){
            _speed_PID_right.set_Pgain_only(_speed_P_rev);
            _speed_PID_left.set_Pgain_only(_speed_P_rev);
        }
    }
    if(!_speed_PID_right.get_controller_on()){
        _speed_PID_right.set_controller_on(PID_ON);
        _speed_PID_right.set_Pgain_only(_speed_P_rev);
        if(set_speed_right < 0.0){
            _speed_PID_right.set_controller_dir(PID_REVERSE);
        }
        else{
            _speed_PID_right.set_controller_dir(PID_DIRECT);
        }
        if((set_speed_left < 0.0) && (set_speed_right < 0.0)){
            _speed_PID_right.set_Pgain_only(_speed_P_rev);
            _speed_PID_left.set_Pgain_only(_speed_P_rev);
        }
    }

    // Update the set point
    _speed_PID_left.set_set_point(set_speed_left);
    _speed_PID_right.set_set_point(set_speed_right);

    // Update left and right speed PIDs
    float meas_speed_left = _encoder_left->get_smooth_speed_mmps();
    float meas_speed_right = _encoder_right->get_smooth_speed_mmps();
    _speed_PID_left.update(meas_speed_left);
    _speed_PID_right.update(meas_speed_right);

    // If the speed is negative then set motors to run backward
    if(set_speed_left < 0.0){
        _motor_left->run(BACKWARD);
    }
    else{
        _motor_left->run(FORWARD);
    }
    if(set_speed_right < 0.0){
        _motor_right->run(BACKWARD);
    }
    else{
        _motor_right->run(FORWARD);
    }
    _motor_left->setSpeed(int(_speed_PID_left.get_output()));
    _motor_right->setSpeed(int(_speed_PID_right.get_output()));


    if(meas_speed_left > (set_speed_left - _speed_tol)  &&
       meas_speed_left < (set_speed_left + _speed_tol)){
        ctrl_state = MOVE_CONTROL_LEFT_COMPLETE;
    }
    if(meas_speed_right > (set_speed_right - _speed_tol)  &&
       meas_speed_right < (set_speed_right + _speed_tol)){
        if(ctrl_state == MOVE_CONTROL_LEFT_COMPLETE){
            ctrl_state = MOVE_CONTROL_COMPLETE;
        }
        else{
            ctrl_state = MOVE_CONTROL_RIGHT_COMPLETE;
        }
    }
    return ctrl_state;
}

//----------------------------------------------------------------------------
EMoveControlState MoveController::to_position(float set_pos_left,
                                              float set_pos_right){
    _pos_ctrl_state = MOVE_CONTROL_INCOMPLETE;

    if(!_pos_PID_left.get_controller_on()){
        _pos_PID_left.set_controller_on(PID_ON);
        _speed_PID_left.set_controller_on(PID_ON);
    }
    if(!_pos_PID_right.get_controller_on()){
        _pos_PID_right.set_controller_on(PID_ON);
        _speed_PID_right.set_controller_on(PID_ON);
    }

    // Check if the set point passed to the function has changed
    int32_t checkSetPointL =  round(set_pos_left/_encoder_left->get_mm_per_count());
    if(checkSetPointL != _set_point_rel_counts_left){
        _set_point_rel_counts_left = checkSetPointL;
        _start_encoder_count_left = _encoder_left->get_count();
        _pos_PID_left.set_set_point(float(_set_point_rel_counts_left));
    }

    int32_t checkSetPointR =  round(set_pos_right/_encoder_right->get_mm_per_count());
    if(checkSetPointR != _set_point_rel_counts_right){
        _set_point_rel_counts_right = checkSetPointR;
        _start_encoder_count_right = _encoder_right->get_count();
        _pos_PID_right.set_set_point(float(_set_point_rel_counts_right));
    }

    // Update the relative count and send it to the PIDs
    _relative_count_left = _encoder_left->get_count()-_start_encoder_count_left;
    _pos_PID_left.update(_relative_count_left);
    _relative_count_right = _encoder_right->get_count()-_start_encoder_count_right;
    _pos_PID_right.update(_relative_count_right);

    // Update the speed PIDs
    _speed_PID_left.update(_encoder_left->get_smooth_speed_mmps());
    _speed_PID_right.update(_encoder_right->get_smooth_speed_mmps());

    // Check that the PID is sending a signal above the min speed
    if(round(abs(_pos_PID_left.get_output())) < _pos_PID_min_speed){
        if(_pos_PID_left.get_output() < 0.0){
            _pos_PID_left.set_output(-1.0*_pos_PID_min_speed);
        }
        else{
            _pos_PID_left.set_output(_pos_PID_min_speed);
        }
    }
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
    if(_relative_count_left < (_set_point_rel_counts_left-_pos_tol)){
        _speed_PID_left.set_set_point(_pos_PID_left.get_output());
        _speed_PID_left.set_controller_dir(PID_DIRECT);
        _motor_left->run(FORWARD);
        _motor_left->setSpeed(int(_speed_PID_left.get_output()));
    }
    else if(_relative_count_left > (_set_point_rel_counts_left+_pos_tol)){
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
        _pos_ctrl_state = MOVE_CONTROL_LEFT_COMPLETE;

    }

    if(_relative_count_right < (_set_point_rel_counts_right-_pos_tol)){
        _speed_PID_right.set_set_point(_pos_PID_right.get_output());
        _speed_PID_right.set_controller_dir(PID_DIRECT);
        _motor_right->run(FORWARD);
        _motor_right->setSpeed(int(_speed_PID_right.get_output()));
    }
    else if(_relative_count_right > (_set_point_rel_counts_right+_pos_tol)){
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
        if(_pos_ctrl_state == MOVE_CONTROL_LEFT_COMPLETE){
            _pos_ctrl_state = MOVE_CONTROL_COMPLETE;
        }
        else{
            _pos_ctrl_state = MOVE_CONTROL_RIGHT_COMPLETE;
        }
    }

    return _pos_ctrl_state;
}

EMoveControlState MoveController::to_dist_ctrl_pos(float set_dist_left,
                                                   float set_dist_right){
    return to_position(set_dist_left,set_dist_right);
}

EMoveControlState MoveController::turn_to_angle_ctrl_pos(float set_angle){
    float arc_leng = set_angle*wheel_data.circ_per_degree;
    return to_position(-1.0*arc_leng,arc_leng);
}
