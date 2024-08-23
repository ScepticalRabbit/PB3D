//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskPounce.h"

TaskPounce::TaskPounce(CollisionManager* collision,
                       MoodManager* mood,
                       TaskManager* task,
                       MoveManager* move,
                       Speaker* speaker){
    _collision_manager = collision;
    _mood_manager = mood;
    _task_manager = task;
    _move_manager = move;
    _speaker = speaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskPounce::begin(){
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskPounce::update(){
    if(!_enabled){return;}

    if(_task_manager->get_new_task_flag()){
        _start_all = true;
    }
}

//---------------------------------------------------------------------------
// Pounce! - called during the main during decision tree
void TaskPounce::seek_and_pounce(){
    if(!_enabled){return;}

    if(_start_all){
        Serial.println("START ALL.");
        _start_all = false;
        _start();
    }

    if(_state != _prev_state){
        _prev_state = _state;
        Serial.print("STATE = ");
        if(_state == POUNCE_SEEK){Serial.print("SEEK");}
        else if(_state == POUNCE_LOCKON){Serial.print("LOCK");}
        else if(_state == POUNCE_RUN){Serial.print("RUN");}
        else if(_state == POUNCE_REALIGN){Serial.print("REALIGN");}
        else{Serial.print("UNKNOWN");}
        Serial.println();
    }

    //-------------------------------------------------------------------------
    // POUNCE DECISION TREE
    switch(_state){
        case POUNCE_SEEK:
            _seek_target();
            break;
        case POUNCE_LOCKON:
            _lock_on();
            break;
        case POUNCE_RUN:
            _run_to_target();
            break;
        case POUNCE_REALIGN:
            _realign();
            break;
        default:
            _move_manager->stop();
            break;
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void TaskPounce::reset(){
    _start();
    _start_all = true;
}

void TaskPounce::collision_reset_to_realign(){
    if(_state == POUNCE_RUN){
        _realign_start = true;
        _state = POUNCE_REALIGN;
    }
}

void TaskPounce::set_realign_cent(int16_t angle){
    _realign_angle_cent = angle;
    _realign_ang_min = _realign_angle_cent-_realign_angle_dev;
    _realign_ang_max = _realign_angle_cent+_realign_angle_dev;
}

//---------------------------------------------------------------------------
// START
void TaskPounce::_start(){
    _state = POUNCE_SEEK;

    _seek_start = true;
    _move_manager->reset_look();

    _lock_start = true;

    _run_start = true;

    _realign_start = true;
    set_realign_cent(180);
}

//---------------------------------------------------------------------------
// SEEK
void TaskPounce::_seek_target(){
if(_seek_start){
    _seek_start = false;
    _move_manager->reset_look();
    _move_manager->reset_PIDs();
    _collision_manager->set_enabled_flag(false);
}
_collision_manager->set_enabled_flag(false); // Disable collision detection

// TASK LEDS
uint8_t seekCol = 1;
if(_move_manager->get_look_curr_ang_ind() == 0){
    //_task_manager->task_LED_CSV(_seek_colour,_seek_colour,255,_low_saturation,255,255);
    _task_manager->task_LED_CSV(_seek_colour,_seek_colour,255,_low_saturation,0,255);
}
else if(_move_manager->get_look_curr_ang_ind() == 1){
    _task_manager->task_LED_CSV(_seek_colour,_seek_colour,_low_saturation,_low_saturation,255,255);
}
else if(_move_manager->get_look_curr_ang_ind() == 2){
    //_task_manager->task_LED_CSV(_seek_colour,_seek_colour,_low_saturation,255,255,255);
    _task_manager->task_LED_CSV(_seek_colour,_seek_colour,_low_saturation,255,255,0);
}
else if(_move_manager->get_look_curr_ang_ind() == 3){
    _task_manager->task_LED_CSV(_seek_colour,_seek_colour,_low_saturation,_low_saturation,255,255);
}
else{
    _task_manager->task_LED_off();
}

// Move to different angles and take measurements.
if(!_move_manager->get_look_finished()){
    _move_manager->look_around();

    if(_meas_index < _meas_num_vals){
    // Get to the set point and start the measurement timer
    if(!_meas_complete){
        if(_move_manager->get_pos_PID_attained_set_point()){
        //Serial.print("SP ATTAINED for "),Serial.println(_meas_index);
        _meas_complete = true;
        _meas_timer.start(_meas_pre_pause_time);
        }
        else if(_move_manager->look_is_paused()){
        //Serial.print("SP TIMEOUT for "),Serial.println(_meas_index);
        _meas_complete = true;
        _meas_timer.start(_meas_pre_pause_time);
        }
    }

    // If measurement timer is finished take a measurement
    if(_meas_complete && _meas_timer.finished()){
        _meas_timer.start(_meas_interval);
        int16_t sample = _collision_manager->get_laser_range(LASER_CENTRE);
        _meas_sum_for_avg = _meas_sum_for_avg + sample;

        Serial.print("Sample "), Serial.print(_meas_count_for_avg);
        Serial.print(" = "), Serial.print(sample), Serial.print(" mm");
        Serial.println();

        _meas_count_for_avg++;

        if(_meas_count_for_avg >= _meas_num_vals){
        _meas_array[_meas_index] = _meas_sum_for_avg/_meas_num_vals;

        Serial.print("Measurement Avg. "),Serial.print(_meas_index);
        Serial.print(" = "), Serial.print(_meas_array[_meas_index]), Serial.print(" mm");
        Serial.println();

        _meas_complete = false;
        _meas_index++;

        _meas_sum_for_avg = 0;
        _meas_count_for_avg = 0;

        // Measurement complete - force move to next pos
        _move_manager->force_look_move();
        }
    }
    }
}
else{ // EXIT CONDITION: LOOK FINISHED, TO NEXT STATE
    _meas_index = 0;
    _state = POUNCE_LOCKON; // Move to next state
    _move_manager->reset_look();
}
}

//---------------------------------------------------------------------------
// LOCK
void TaskPounce::_lock_on(){
    if(_lock_start){
        _lock_start = false;

        // Decide on target
        _lock_valid_range_count = 0;
        for(int8_t ii=0; ii < _meas_num_vals; ii++){
        if((_meas_array[ii] >= _lock_limit_range_min)&&(_meas_array[ii] <= _lock_limit_range_max)){
            _lock_valid_range_count++;

            if(_meas_array[ii] < _lock_valid_range_min){
            _lock_valid_range_min = _meas_array[ii];
            _lock_valid_range_min_ind = ii;
            }
            if(_meas_array[ii] > _lock_valid_range_max){
            _lock_valid_range_max = _meas_array[ii];
            _lock_valid_range_max_ind = ii;
            }
        }
        }
        // LOCK ON ANGLE/RANGE
        _lock_on_ang = _meas_angles[_lock_valid_range_min_ind];
        _lock_on_range = float(_meas_array[_lock_valid_range_min_ind]);
        _lock_on_timer.start(_lock_spool_up_time);  // Start timer

        _move_manager->reset_PIDs(); // Reset PIDs
        _collision_manager->set_enabled_flag(false); // Disable collisition detection

        // DEBUG: Lock on start
        Serial.println("LOCK ON: Start");
    }
    _collision_manager->set_enabled_flag(false); // Disable collision detection

    // Decide on a target
    // 1) If all ranges less than min range - REALIGN
    // 2) If all ranges greater than max range - GO TO CLOSEST
    // 3) Go through valid ranges and go to the nearest one - GO TO CLOSEST
    if(_lock_valid_range_count == 0){
        _state = POUNCE_REALIGN;
    }
    else{
        // Spool up - flash lights and wag tail - TODO
        _task_manager->task_LED_CSV(_lock_colour,_lock_colour,_low_saturation,_low_saturation,255,255);

        // Turn to target
        _move_manager->turn_to_angle_ctrl_pos(_lock_on_ang);

        // EXIT CONDITION: SET POINT REACHED
        if(_move_manager->get_pos_PID_attained_set_point()){
        _state = POUNCE_RUN;
        }

        // EXIT CONDITION: TIMEOUT
        if(_lock_on_timer.finished()){
        _state = POUNCE_RUN;
        }
    }
}

//---------------------------------------------------------------------------
// RUN
void TaskPounce::_run_to_target(){
    if(_run_start){
        _run_start = false;

        // Re-calc timeout based on the measured range
        _run_timeout = int16_t(((_lock_on_range-float(_run_range_limit))/_run_speed)*1000.0)+500;
        _run_timer.start(_run_timeout); // Start timer

        // Calculate encoder counts to get to target
        int32_t runEncCounts = int32_t((_lock_on_range-float(_run_range_limit))/(_move_manager->get_encoder_mm_per_count()));
        int32_t encAvgCounts = (_move_manager->get_encoder_count_left()+_move_manager->get_encoder_count_right())/2;
        _run_end_encoder_count = encAvgCounts+runEncCounts;

        _collision_manager->set_enabled_flag(true); // Re-enable collision detection
        // DEBUG: Run to start
        Serial.println("RUN: Start");
        Serial.print("RUN: Timeout = ");
        Serial.println(_run_timeout);
        Serial.print("RUN: Enc. Counts = ");
        Serial.println(runEncCounts);
    }

    // TASK LEDS
    _task_manager->task_LED_CSV(_run_colour,_run_colour,_low_saturation,_low_saturation,255,255);

    // EXIT CONDITION: Found target
    if(_collision_manager->get_laser_range(LASER_CENTRE) <= _run_range_limit){
        _state = POUNCE_REALIGN;
        Serial.println("RUN END: US Range");
    }
    else if((_move_manager->get_encoder_count_left() >= _run_end_encoder_count) || (_move_manager->get_encoder_count_right() >= _run_end_encoder_count)){
        _state = POUNCE_REALIGN;
        Serial.println("RUN END: Enc Counts");
    }
    else{ // Go forward fast
        _move_manager->forward(_run_speed);
    }

    // EXIT CONDITION: TIMEOUT
    if(_run_timer.finished()){
        _state = POUNCE_REALIGN;
        Serial.println("RUN END: Timer");
    }
}

//---------------------------------------------------------------------------
// REALIGN
void TaskPounce::_realign(){
    if(_realign_start){
        _realign_start = false;

        // Set to the initial state
        _realign_state = 0;

        // Generate angle and start timers
        _realign_angle = float(random(_realign_ang_min,_realign_ang_max));
        _realign_timer.start(_realign_pre_pause_time);

        _collision_manager->set_enabled_flag(false); // Disable collision detection
        _move_manager->reset_PIDs();  // Reset move PIDs

        // DEBUG: Run to start
        Serial.println("REALIGN: Start, Pre-pause");
    }
    _collision_manager->set_enabled_flag(false); // Disable collision detection

    // TASK LEDS
    _task_manager->task_LED_CSV(_realign_colour,_realign_colour,_low_saturation,_low_saturation,255,255);

    // Move to the randomly generate angle to reorient
    if(_realign_state == 0){ // Pre-pause
        _move_manager->stop();

        if(_realign_timer.finished()){
        _realign_timer.start(_realignTimeout);
        _realign_state++;
        Serial.print("REALIGN: Moving to ");
        Serial.print(_realign_angle);
        Serial.println(" deg");
        }
    }
    else if(_realign_state == 1){// Move to angle
        _move_manager->turn_to_angle_ctrl_pos(_realign_angle);

        // If angle is obtained or timeout go back to the start
        if(_move_manager->get_pos_PID_attained_set_point() || _realign_timer.finished()){
        _realign_timer.start(_realign_post_pause_time);
        _realign_state++;
        Serial.println("REALIGN: Post pause");
        }
    }
    else if(_realign_state == 2){// Post-pause
        _move_manager->stop();

        if(_realign_timer.finished()){
        _realign_timer.start(_realignTimeout);
        _realign_state++;

        _state = POUNCE_SEEK;
        reset(); // Go back to start
        Serial.println("REALIGN: Finished, Reset");
        }
    }
    else{
        _move_manager->stop();
    }
    /*
    if(_realignPrePauseTimer.finished()){
        _move_manager->turnToAnglePosCtrl(_realign_angle);

        // If angle is obtained or timeout go back to the start
        if(_move_manager->get_pos_PID_attained_set_point() || _realign_timer.finished()){
        _realignPostPauseTimer.start(_realign_post_pause_time);
        }
    }
    else if(_realignPrePauseTimer.finished() && _realignPostPauseTimer.finished()){
        _state = POUNCE_SEEK;
        reset(); // Go back to start
        Serial.println("REALIGN: Reset");
    }
    else{
        // Stop moving and wait for motors/momentum to settle
        _move_manager->stop();
    }
    */
}