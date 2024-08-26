//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveManager.h"
#include "MoveBasic.h"

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
    _move_compound_code = random(0,_move_compound_count);
    _move_update_time = random(_move_update_min_time,_move_update_max_time);
    _move_timer.start(_move_update_time);
    _submove_timer.start(0);
    _look_timer.start(0);
    _timeout_timer.start(0);
  }

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void MoveManager::update_move(){
    if(!_enabled){return;}

    if(_move_timer.finished()){
        _move_compound_code = random(0,_move_compound_count);
        _update_compound_move();
    }
}

void MoveManager::update_move(int8_t inMoveType){
    if(!_enabled){return;}

    if(_move_timer.finished()){
        _move_compound_code = inMoveType;
        _update_compound_move();
    }
}

//---------------------------------------------------------------------------
// GO
void MoveManager::go(){
    if(!_enabled){return;}

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
void MoveManager::set_power_by_diff(int8_t inDiff){
    _cur_forward_power= _def_forward_power+inDiff;
    _cur_back_power = _def_back_power+inDiff;
    _cur_turn_power = _def_turn_power+inDiff;
  }

void MoveManager::set_speed_by_col_code(bool obstacleClose){
    if(obstacleClose){_speed_col_fact = _speed_col_true;}
    else{_speed_col_fact = _speed_col_false;}
    _update_speed();
}

void MoveManager::set_move_control(int8_t inMoveControl){
    if(inMoveControl == MOVE_CONTROL_SPEED){
        // Use PIDs to control speed in mm/s
        _move_control_code = MOVE_CONTROL_SPEED;
    }
    else{
        // Bypass PIDs and directly set motor PWM output between 0-255
        _move_control_code = MOVE_CONTROL_POWER;
    }
}

void MoveManager::change_circ_dir(){
    if(_spiral_direction == MOVE_B_LEFT){
        _spiral_direction = MOVE_B_RIGHT;
    }
    else{
        _spiral_direction = MOVE_B_LEFT;
    }
    if(_circle_direction == MOVE_B_LEFT){
        _circle_direction = MOVE_B_RIGHT;
    }
    else{
        _circle_direction = MOVE_B_LEFT;
    }
}

void MoveManager::set_speed_by_mood_fact(float inFact){
    _speed_mood_fact = inFact;
    _update_speed();
}

//---------------------------------------------------------------------------
// CALCULATORS
uint16_t MoveManager::calc_timeout(float inSpeed, float inDist){
    float absSpeed = abs(inSpeed);
    float absDist = abs(inDist);
    float timeToVel = absSpeed/_speed_timeout_accel; // seconds
    float timeToDist = sqrt((2*absDist)/_speed_timeout_accel); //seconds
    float timeout = 0.0;
    if(timeToDist < timeToVel){ // Finish moving before we finish acceleratin
        timeout = timeToDist;
    }
    else{ // Finish moving after we finish accelerating
        float timeAtConstVel = (absDist-0.5*_speed_timeout_accel*timeToVel*timeToVel)/absSpeed;
        timeout = timeAtConstVel+timeToVel;
    }
    return uint16_t(_speed_timeout_SF*timeout*1000.0); // milliseconds
}

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


void MoveManager::forward_right(){
    _update_basic_move(MOVE_B_FORRIGHT);
    _move_basic.forward_right();
}

void MoveManager::forward_right_diff_frac(float diff_frac){
    _update_basic_move(MOVE_B_FORRIGHT);
    _move_basic.forward_right_diff_frac(diff_frac);
}


//----------------------------------------------------------------------------
// Move Circle
void MoveManager::circle(){
    _move_circle.circle();
}

//============================================================================
// ENCODER/PID CONTROLLED MOVEMENT FUNCTIONS
//============================================================================

// NOTE: position can be negative to move backwards
void MoveManager::to_dist_ctrl_pos(float setDist){
    //_update_basic_move(MOVE_B_TODIST_CPOS);
    _move_controller.to_position(setDist,setDist);
}

void MoveManager::to_dist_ctrl_pos(float setDistL, float setDistR){
    //_update_basic_move(MOVE_B_TODIST_CPOS);
    _move_controller.to_position(setDistL,setDistR);
}

void MoveManager::turn_to_angle_ctrl_pos(float setAngle){
    //_update_basic_move(MOVE_B_TOANG_CPOS);
    float arcLeng = setAngle*_wheel_circ_ang;
    _move_controller.to_position(-1.0*arcLeng,arcLeng);
}

int8_t MoveManager::to_dist_ctrl_speed(float speedL, float speedR,
    float setDistL, float setDistR){
    int8_t isComplete = 0;
    //_update_basic_move(MOVE_B_TODIST_CSpeed);

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
            _move_controller.at_speed(abs(speedL),abs(speedR));
        }
        else{
            isComplete = 1;
            stop_no_update();
        }
        }
        else if((setDistL < 0.0) && (setDistR > 0.0)){ // Turn left
            if((_encoder_left->get_count() >= _end_encoder_count_left)||(_encoder_right->get_count() <= _end_encoder_count_right)){
                _move_controller.at_speed(-1.0*abs(speedL),abs(speedR));
            }
            else{
                isComplete = 1;
                stop_no_update();
            }
        }
        else if((setDistL > 0.0) && (setDistR < 0.0)){
            if((_encoder_left->get_count() <= _end_encoder_count_left)||(_encoder_right->get_count() >= _end_encoder_count_right)){
                _move_controller.at_speed(abs(speedL),-1.0*abs(speedR));
            }
            else{
                isComplete = 1;
                stop_no_update();
            }
        }
        else if((setDistL < 0.0) && (setDistR < 0.0)){ // Turn right
            if((_encoder_left->get_count() >= _end_encoder_count_left)||(_encoder_right->get_count() >= _end_encoder_count_right)){
                _move_controller.at_speed(-1.0*abs(speedL),-1.0*abs(speedR));
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

int8_t MoveManager::to_dist_ctrl_speed(float setDist){
    int8_t isComplete = 0;
    if(setDist < 0.0){
        isComplete = to_dist_ctrl_speed(_cur_back_speed,_cur_back_speed,setDist,setDist);
    }
    else{
        isComplete = to_dist_ctrl_speed(_cur_forward_speed,_cur_forward_speed,setDist,setDist);
    }
    return isComplete;
}

int8_t MoveManager::turn_to_angle_ctrl_speed(float setAngle){
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


//============================================================================
// COMPOUND MOVEMENT FUNCTIONS
//============================================================================
// NOTE: these two functions already work with the default speed
// ADD: compound move code to each of these

//----------------------------------------------------------------------------
// MOVE WIGGLE LEFT/RIGHT
void MoveManager::wiggle(){
    wiggle(_wiggle_def_left_dur,_wiggle_def_right_dur);
}

void MoveManager::wiggle(uint16_t leftDur, uint16_t rightDur){
    // Update the wiggle direction if needed
    if(_submove_timer.finished()){
        stop(); // Stop the motors because we are going to switch direction
        if(_wiggle_left_flag){
        // If we are turning left, switch to right
        _wiggle_left_flag = false;
        _wiggle_curr_dur = rightDur;
        }
        else{
        // If we are turning right, switch to left
        _wiggle_left_flag = true;
        _wiggle_curr_dur = leftDur;
        }
        _submove_timer.start(_wiggle_curr_dur);
    }

    if(_wiggle_left_flag){
        left();
    }
    else{
        right();
    }
}

//----------------------------------------------------------------------------
// MOVE FORWARD/BACK
void MoveManager::forward_back(){
    forward_back(_FB_def_forward_dur,_FB_def_back_dur);
}

void MoveManager::forward_back(uint16_t forwardDur, uint16_t backDur){
    // Update the forward/back direction if needed
    if(_submove_timer.finished()){
        stop(); // Stop the motors because we are going to switch direction
        if(_FB_forward_flag){
        // If we are going forward, switch to back
        _FB_forward_flag = false;
        _FB_curr_dur = backDur;
        }
        else{
        // If we are going back, switch to forward
        _FB_forward_flag = true;
        _FB_curr_dur = forwardDur;
        }
        _submove_timer.start(_FB_curr_dur);
    }

    if(_FB_forward_flag){
        forward();
    }
    else{
        back();
    }
}

//----------------------------------------------------------------------------
// MOVE SPIRAL
void MoveManager::spiral(){
    spiral(_spiral_direction);
}
void MoveManager::spiral(int8_t turnDir){
    if(_move_control_code == MOVE_CONTROL_SPEED){
        spiral_speed(turnDir);
    }
    else{
        spiral_power(turnDir);
    }
}

void MoveManager::spiral_speed(int8_t turnDir){
    if(_spiral_start || (_move_compound_code != MOVE_C_SPIRAL)){
        // Reset the flag so we don't re-init,
        _spiral_start = false;
        _move_compound_code = MOVE_C_SPIRAL;
        // Calculate the speed/time slope for the linear increase of speed
        // for the slow wheel
        _init_spiral_speed_diff = _cur_forward_speed-_spiral_min_speed;
        _cur_spiral_speed_diff = _init_spiral_speed_diff;
        _spiral_slope = _init_spiral_speed_diff/float(_spiral_duration);
        // Start the spiral timer
        _submove_timer.start(_spiral_duration);
    }

    // Calculate the current speed for the slope wheel based on the timer
    _spiral_curr_time = _submove_timer.get_time();
    _cur_spiral_speed_diff = _init_spiral_speed_diff - _spiral_slope*float(_spiral_curr_time);

    // Check if we are increasing the speed of the slow wheel above the fast one
    if(_cur_spiral_speed_diff>_cur_forward_speed){
        _cur_spiral_speed_diff = _cur_forward_speed-_min_speed;
    }

    // If the spiral time is finished then set the flag to restart the spiral
    if(_submove_timer.finished()){
        _spiral_start = true;
    }
    else{
        if(turnDir == MOVE_B_LEFT){
            _move_basic.forward_left_speed(_cur_forward_speed,_cur_spiral_speed_diff);
        }
        else{
            _move_basic.forward_right_speed(_cur_forward_speed,_cur_spiral_speed_diff);
        }
    }
}

void MoveManager::spiral_power(int8_t turnDir){
    if(_spiral_start || (_move_compound_code != MOVE_C_SPIRAL)){
        // Reset the flag so we don't re-init,
        _spiral_start = false;
        _move_compound_code = MOVE_C_SPIRAL;
        // Calculate the speed/time slope for the linear increase of speed
        // for the slow wheel
        _init_spiral_speed_diff_power = _cur_forward_power-_spiral_min_power;
        _cur_spiral_speed_diff_power = _init_spiral_speed_diff_power;
        _spiral_slope_power = float(_init_spiral_speed_diff_power)/float(_spiral_duration);
        // Start the spiral timer
        _submove_timer.start(_spiral_duration);
    }

    _spiral_curr_time = _submove_timer.get_time();
    _cur_spiral_speed_diff_power = round(float(_init_spiral_speed_diff) - _spiral_slope_power*float(_spiral_curr_time));

    if(_cur_spiral_speed_diff_power>_cur_forward_power){
        _cur_spiral_speed_diff_power = _cur_forward_power-_spiral_min_power;
    }

    if(_submove_timer.finished()){
        _spiral_start = true;
    }
    else{
        if(turnDir == MOVE_B_LEFT){
            _move_basic.forward_left_power(_cur_forward_power,_cur_spiral_speed_diff_power);
        }
        else{
            _move_basic.forward_right_power(_cur_forward_power,_cur_spiral_speed_diff_power);
        }
    }
}

//----------------------------------------------------------------------------
// MOVE ZIG/ZAG
void MoveManager::zig_zag(){
    if(_zz_turn_flag){
        if(!_submove_timer.finished()){
        if(_zz_turn_dir == MOVE_B_LEFT){
            if(_move_control_code == MOVE_CONTROL_SPEED){
                _move_basic.forward_left_speed(_cur_turn_speed,_zz_turn_diff_speed);
            }
            else{
                _move_basic.forward_left_power(_cur_turn_power,_zz_turn_diff_power);
            }
        }
        else{
            if(_move_control_code == MOVE_CONTROL_SPEED){
                _move_basic.forward_right_speed(_cur_turn_speed,_zz_turn_diff_speed);
            }
            else{
                _move_basic.forward_right_power(_cur_turn_power,_zz_turn_diff_power);
            }
        }
        }
        else{
        if(_zz_turn_dir == MOVE_B_LEFT){
            _zz_turn_dir = MOVE_B_RIGHT;
            _zz_turn_duration = _zz_right_turn_dur;
        }
        else{
            _zz_turn_dir = MOVE_B_LEFT;
            _zz_turn_duration = _zz_left_turn_dur;
        }
        _zz_turn_flag = false;
        _zz_straight_flag = true;
        _submove_timer.start(_zz_straight_duration);
        }
    }

    if(_zz_straight_flag){
        if(!_submove_timer.finished()){
            forward();
        }
        else{
            _zz_turn_flag = true;
            _zz_straight_flag = false;
            _submove_timer.start(_zz_turn_duration);
        }
    }
}

//----------------------------------------------------------------------------
// MOVE LOOK AROUND
void MoveManager::look_around(){
    if(_look_start_flag){
        //Serial.println("LOOK START.");
        _move_compound_code = MOVE_C_LOOK;
        _look_start_flag = false;
        _look_timer.start(_look_move_time);
        _look_cur_ang = 0;
        _look_move_switch = true;
        _move_controller.reset();
    }
    else{
        if(_look_timer.finished()){
        _look_move_switch = !_look_move_switch;
        //Serial.print("LOOK TIMER FINISHED: ");
        if(_look_move_switch){
            //Serial.println("START. Ang++");
            _look_timer.start(_look_move_time);
            _look_cur_ang++;

            /*if(_look_cur_ang >= _look_num_angs){
            _look_cur_ang = 0;
            }*/
        }
        else{
            //Serial.println("PAUSE.");
            _look_timer.start(_look_pause_time);
        }
        }
    }

    if(_look_move_switch && (_look_cur_ang <= _look_num_angs)){
        float moveAng = 0.0;
        if(_look_cur_ang == 0){
        moveAng = _look_angles[_look_cur_ang];
        }
        else{
        moveAng = _look_angles[_look_cur_ang] - _look_angles[_look_cur_ang-1];
        }
        turn_to_angle_ctrl_pos(moveAng);
    }
    else{
        stop();
    }
}

void MoveManager::force_look_move(){
    //Serial.println("FUNC: force look move. Ang++");
    _look_move_switch = true;
    _look_cur_ang++;
    _look_timer.start(_look_move_time);
    _move_controller.reset();
}

void MoveManager::reset_look(){
    //Serial.println("FUNC: reset look.");
    _look_start_flag = true;
    _look_cur_ang = 0;
    _move_controller.reset();
}


//----------------------------------------------------------------------------
// PRIVATE HELPER FUNCTIONS
void MoveManager::_update_basic_move(EMoveBasic move){
    if(_move_basic_code != move){
        _move_basic_code = move;
        _encoder_count_start = true;
        _start_encoder_count_left = _encoder_left->get_count();
        _start_encoder_count_right = _encoder_right->get_count();
        _move_controller.reset();
    }
}

//----------------------------------------------------------------------------
void MoveManager::_update_compound_move(){
    _move_update_time = random(_move_update_min_time,_move_update_max_time);

    if(_move_compound_code == MOVE_C_SPIRAL){
        _spiral_start = true;
        _spiral_direction = random(0,2);
        _move_update_time = _spiral_duration;
    }
    else if(_move_compound_code == MOVE_C_CIRCLE){
        _circle_direction = random(0,2);
    }
    else if(_move_compound_code == MOVE_C_LOOK){
        _look_start_flag = true;
        _move_update_time = _look_tot_time;
    }

    // Restart timers
    _move_timer.start(_move_update_time);
    _submove_timer.start(0);
}

//----------------------------------------------------------------------------
void MoveManager::_update_speed(){
    _cur_forward_speed = constrain(_def_forward_speed*_speed_mood_fact*_speed_col_fact,_min_speed,_max_speed);
    _cur_back_speed = -1.0*constrain(fabs(_def_back_speed*_speed_mood_fact*_speed_col_fact),_min_speed,_max_speed);
    _cur_turn_speed = constrain(_def_turn_speed*_speed_mood_fact*_speed_col_fact,_min_speed,_max_speed);
}
