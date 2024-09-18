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
    _pos_ctrl_state = MOVE_CONTROL_START;
    _pos_PID_left.set_set_point(0.0);
    _pos_PID_right.set_set_point(0.0);
    _pos_PID_left.set_output(0.0);
    _pos_PID_right.set_output(0.0);
    _pos_PID_left.set_controller_on(PID_OFF);
    _pos_PID_right.set_controller_on(PID_OFF);

    _speed_ctrl_state = MOVE_CONTROL_START;
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
    int32_t set_point_count_left =  round(set_pos_left /
                                _encoder_left->get_mm_per_count());
    int32_t set_point_count_right =  round(set_pos_right /
                                    _encoder_right->get_mm_per_count());

    if(set_point_count_left != encoder_state.get_counts_to_end_left() ||
       set_point_count_right != encoder_state.get_counts_to_end_right()){
        _pos_ctrl_state = MOVE_CONTROL_START;
        _pos_PID_left.set_set_point(float(set_point_count_left));
        _pos_PID_right.set_set_point(float(set_point_count_right));
    }

    if(_pos_ctrl_state == MOVE_CONTROL_START){
        _pos_ctrl_state = MOVE_CONTROL_INCOMPLETE;
        encoder_state.start_with_diff(_encoder_left->get_count(),
                                      _encoder_right->get_count(),
                                      set_point_count_left,
                                      set_point_count_right);
    }

    if(!_pos_PID_left.get_controller_on()){
        _pos_PID_left.set_controller_on(PID_ON);
        _speed_PID_left.set_controller_on(PID_ON);
    }
    if(!_pos_PID_right.get_controller_on()){
        _pos_PID_right.set_controller_on(PID_ON);
        _speed_PID_right.set_controller_on(PID_ON);
    }

    int32_t counts_from_start_left =
            encoder_state.counts_from_start_left(_encoder_left->get_count());
    int32_t counts_from_start_right =
            encoder_state.counts_from_start_right(_encoder_right->get_count());

    _pos_PID_left.update(counts_from_start_left);
    _pos_PID_right.update(counts_from_start_right);

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
    if(counts_from_start_left <
       (encoder_state.get_counts_to_end_left()-_pos_tol)){
        _speed_PID_left.set_set_point(_pos_PID_left.get_output());
        _speed_PID_left.set_controller_dir(PID_DIRECT);
        _motor_left->run(FORWARD);
        _motor_left->setSpeed(int(_speed_PID_left.get_output()));
    }
    else if(counts_from_start_left >
            (encoder_state.get_counts_to_end_left()+_pos_tol)){
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

    if(counts_from_start_right <
       (encoder_state.get_counts_to_end_right()-_pos_tol)){
        _speed_PID_right.set_set_point(_pos_PID_right.get_output());
        _speed_PID_right.set_controller_dir(PID_DIRECT);
        _motor_right->run(FORWARD);
        _motor_right->setSpeed(int(_speed_PID_right.get_output()));
    }
    else if(counts_from_start_right >
            (encoder_state.get_counts_to_end_right()+_pos_tol)){
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

EMoveControlState MoveController::to_dist_ctrl_speed(float speed_left,
                                                     float speed_right,
                                                     float set_dist_left,
                                                     float set_dist_right){

    // If the set distance changes outside tolerance force update
    if(!((set_dist_left >= (_to_dist_set_pt_left-_to_dist_tol)) &&
         (set_dist_left <= (_to_dist_set_pt_left+_to_dist_tol)))){
        _to_dist_set_pt_left = set_dist_left;
        reset();
        _speed_ctrl_state = MOVE_CONTROL_START;
    }
    if(!((set_dist_right >= (_to_dist_set_pt_right-_to_dist_tol)) &&
         (set_dist_right <= (_to_dist_set_pt_right+_to_dist_tol)))){
        _to_dist_set_pt_right = set_dist_right;
        reset();
        _speed_ctrl_state = MOVE_CONTROL_START;
    }

    if(_speed_ctrl_state == MOVE_CONTROL_START){
        _speed_ctrl_state = MOVE_CONTROL_INCOMPLETE;

        uint16_t timeout_left = calc_timeout(speed_left,set_dist_left);
        uint16_t timeout_right = calc_timeout(speed_right,set_dist_right);
        if(timeout_left > timeout_right){
            _timeout_timer.start(timeout_left);
        }
        else{
            _timeout_timer.start(timeout_right);
        }

        encoder_state.start_with_diff(_encoder_left->get_count(),
                                      _encoder_right->get_count(),
                                      int32_t(set_dist_left /
                                      _encoder_left->get_mm_per_count()),
                                      int32_t(set_dist_right /
                                      _encoder_left->get_mm_per_count()));
    }



    if(_timeout_timer.finished()){
        _stop();
        _speed_ctrl_state = MOVE_CONTROL_TIMEOUT;
        return _speed_ctrl_state;
    }

    if((set_dist_left > 0.0) && (set_dist_right > 0.0)){ // Go forward
        if((_encoder_left->get_count() <= encoder_state.end_count_left())||
        (_encoder_right->get_count() <= encoder_state.end_count_right())){
            at_speed(abs(speed_left),abs(speed_right));
        }
        else{
            _stop();
            _speed_ctrl_state = MOVE_CONTROL_COMPLETE;
            return _speed_ctrl_state;
        }
    }
    else if((set_dist_left < 0.0) && (set_dist_right > 0.0)){ // Turn left
        if((_encoder_left->get_count() >= encoder_state.end_count_left())||
            (_encoder_right->get_count() <= encoder_state.end_count_right())){
            at_speed(-1.0*abs(speed_left),abs(speed_right));
        }
        else{
            _stop();
            _speed_ctrl_state = MOVE_CONTROL_COMPLETE;
            return _speed_ctrl_state;
        }
    }
    else if((set_dist_left > 0.0) && (set_dist_right < 0.0)){
        if((_encoder_left->get_count() <= encoder_state.end_count_left())||
            (_encoder_right->get_count() >= encoder_state.end_count_right())){
            at_speed(abs(speed_left),-1.0*abs(speed_right));
        }
        else{
            _stop();
            _speed_ctrl_state = MOVE_CONTROL_COMPLETE;
            return _speed_ctrl_state;
        }
    }
    else if((set_dist_left < 0.0) && (set_dist_right < 0.0)){ // Turn right
        if((_encoder_left->get_count() >= encoder_state.end_count_left())||
            (_encoder_right->get_count() >= encoder_state.end_count_right())){
            at_speed(-1.0*abs(speed_left),-1.0*abs(speed_right));
        }
        else{
            _stop();
            _speed_ctrl_state = MOVE_CONTROL_COMPLETE;
            return _speed_ctrl_state;
        }
    }
    else{
        _stop();
        _speed_ctrl_state = MOVE_CONTROL_COMPLETE;
        return _speed_ctrl_state;
    }
    
    return _speed_ctrl_state;
}

uint16_t MoveController::calc_timeout(float speed, float dist){
    float abs_speed = fabs(speed);
    float abs_dist = fabs(dist);
    float time_to_vel = abs_speed/_speed_timeout_accel; // seconds
    float time_to_dist = sqrt((2*abs_dist)/_speed_timeout_accel); //seconds
    float timeout = 0.0;

    if(time_to_dist < time_to_vel){ // Finish moving before we finish acceleratin
        timeout = time_to_dist;
    }
    else{ // Finish moving after we finish accelerating
        float timeAtConstVel = (abs_dist-0.5*_speed_timeout_accel*time_to_vel*time_to_vel)/abs_speed;
        timeout = timeAtConstVel+time_to_vel;
    }

    return uint16_t(_speed_timeout_safety_factor*timeout*1000.0); // milliseconds
}

void MoveController::_stop(){
    _motor_left->run(RELEASE);
    _motor_right->run(RELEASE);
    _motor_left->setSpeed(0);
    _motor_right->setSpeed(0);
}
