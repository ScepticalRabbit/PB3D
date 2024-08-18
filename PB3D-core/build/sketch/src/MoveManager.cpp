#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/MoveManager.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveManager.h"

MoveManager::MoveManager(Adafruit_MotorShield* AFMS,
                        Encoder* encL,
                        Encoder* encR){
    _motor_shield = AFMS;
    _encoder_left = encL;
    _encoder_right = encR;
}

//------------------------------------------------------------------------------
void MoveManager::begin(){
    // Start the motor shield object
    _motor_shield->begin();  // create with the default frequency 1.6KHz
    // M1 is the right motor, M2 is the left motor
    _motor_right = _motor_shield->getMotor(1);
    _motor_left = _motor_shield->getMotor(2);
    // Set the speed to start, from 0 (off) to 255 (max  speed)
    _motor_right->setSpeed(_def_forward_power);
    _motor_right->run(FORWARD);
    _motor_right->run(RELEASE);
    _motor_left->setSpeed(_def_forward_power);
    _motor_left->run(FORWARD);
    _motor_left->run(RELEASE);

    // Randomly generate a move type and start the timer
    _move_compound = random(0,_move_compound_count);
    _move_update_time = random(_move_update_min_time,_move_update_max_time);
    _move_timer.start(_move_update_time);
    _submove_timer.start(0);
    _look_timer.start(0);
    _timeout_timer.start(0);

    // Start/setup the encoders
    _encoder_left->begin();
    _encoder_right->begin();
    // Start the speed PIDs
    _speed_PID_left.begin();
    _speed_PID_right.begin();
    _speed_PID_left.set_output_limits(_min_power, 255.0);
    _speed_PID_right.set_output_limits(_min_power, 255.0);
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

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void MoveManager::update_move(){
    if(!_enabled){return;}

    if(_move_timer.finished()){
        _move_compound = random(0,_move_compound_count);
        _update_compound_move();
    }
}

void MoveManager::update_move(int8_t inMoveType){
    if(!_enabled){return;}

    if(_move_timer.finished()){
        _move_compound = inMoveType;
        _update_compound_move();
    }
}

//---------------------------------------------------------------------------
// GO
//---------------------------------------------------------------------------
void MoveManager::go(){
    if(!_enabled){return;}

    if(_move_compound == MOVE_C_ZIGZAG){
        zig_zag();
    }
    else if(_move_compound == MOVE_C_SPIRAL){
        spiral();
    }
    else if(_move_compound == MOVE_C_CIRCLE){
        circle();
    }
    else if(_move_compound == MOVE_C_LOOK){
        look_around();
    }
    else{
        forward();
    }
}

//---------------------------------------------------------------------------
// GET,SET and RESET functions: full implementation
//---------------------------------------------------------------------------
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
        _move_control = MOVE_CONTROL_SPEED;
    }
    else{
        // Bypass PIDs and directly set motor PWM output between 0-255
        _move_control = MOVE_CONTROL_POWER;
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
//---------------------------------------------------------------------------
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
// BASIC MOVEMENT FUNCTIONS - GENERIC (switched by _move_control var)
//---------------------------------------------------------------------------

//----------------------------------------------------------------------------
// MoveManager Stop - same regardless of control mode
void MoveManager::stop(){
    _update_basic_move(MOVE_B_STOP);
    _motor_left->run(RELEASE);
    _motor_right->run(RELEASE);
    _motor_left->setSpeed(0);
    _motor_right->setSpeed(0);
}

void MoveManager::stop_no_update(){
    _motor_left->run(RELEASE);
    _motor_right->run(RELEASE);
    _motor_left->setSpeed(0);
    _motor_right->setSpeed(0);
}

//----------------------------------------------------------------------------
// MoveManager Forward
void MoveManager::forward(){
    if(_move_control == MOVE_CONTROL_SPEED){
        forward_speed(_cur_forward_speed);
    }
    else{
        forward_power(_cur_forward_power);
    }
}
void MoveManager::forward(float inSpeed){
    forward_speed(inSpeed);
}
void MoveManager::forward(uint8_t inPower){
    forward_power(inPower);
}

//----------------------------------------------------------------------------
// Move Back
void MoveManager::back(){
    if(_move_control == MOVE_CONTROL_SPEED){
        back_speed(_cur_back_speed);
    }
    else{
        back_power(_cur_back_power);
    }
}
void MoveManager::back(float inSpeed){
    back_speed(inSpeed);
}
void MoveManager::back(uint8_t inPower){
    back_power(inPower);
}

//----------------------------------------------------------------------------
// Move Left
void MoveManager::left(){
    if(_move_control == MOVE_CONTROL_SPEED){
        left_speed(_cur_turn_speed);
    }
    else{
        left_power(_cur_turn_power);
    }
}
void MoveManager::left(float inSpeed){
    left_speed(inSpeed);
}
void MoveManager::left(uint8_t inPower){
    left_power(inPower);
}

//----------------------------------------------------------------------------
// Move Right
void MoveManager::right(){
    if(_move_control == MOVE_CONTROL_SPEED){
        right_speed(_cur_turn_speed);
    }
    else{
        right_power(_cur_turn_power);
    }
}
void MoveManager::right(float inSpeed){
    right_speed(inSpeed);
}
void MoveManager::right(uint8_t inPower){
    right_power(inPower);
}

//----------------------------------------------------------------------------
// Move Forward Left
void MoveManager::forward_left(){
if(_move_control == MOVE_CONTROL_SPEED){
    forward_left(_cur_forward_speed, _curTurnSpeedDiff);
}
else{
    forward_left(_cur_forward_power, _cur_turn_power_diff);
}
}

void MoveManager::forward_left_diff_frac(float diffFrac){
if(_move_control == MOVE_CONTROL_SPEED){
    float speedDiff = _cur_forward_speed*diffFrac;
    forward_left(_cur_forward_speed, speedDiff);
}
else{
    uint8_t PowerDiff = round(diffFrac*float(_cur_forward_power));
    forward_left(_cur_forward_power, PowerDiff);
}
}

void MoveManager::forward_left(float inSpeedDiff){
    forward_left_speed(_cur_turn_speed,inSpeedDiff);
}
void MoveManager::forward_left(float inSpeed, float inSpeedDiff){
    forward_left_speed(inSpeed,inSpeedDiff);
}

void MoveManager::forward_left(uint8_t inPowerDiff){
    forward_left_power(_cur_turn_power,inPowerDiff);
}
void MoveManager::forward_left(uint8_t inPower, uint8_t inPowerDiff){
    forward_left_power(inPower,inPowerDiff);
}

//----------------------------------------------------------------------------
// Move Forward Right
void MoveManager::forward_right(){
if(_move_control == MOVE_CONTROL_SPEED){
    forward_right(_cur_forward_speed, _curTurnSpeedDiff);
}
else{
    forward_right(_cur_forward_power, _cur_turn_power_diff);
}
}

void MoveManager::forward_right_diff_frac(float diffFrac){
if(_move_control == MOVE_CONTROL_SPEED){
    float speedDiff = _cur_forward_speed*diffFrac;
    forward_right(_cur_forward_speed, speedDiff);
}
else{
    uint8_t PowerDiff = round(diffFrac*float(_cur_forward_power));
    forward_right(_cur_forward_power, PowerDiff);
}
}

void MoveManager::forward_right(float inSpeedDiff){
forward_right_speed(_cur_turn_speed,inSpeedDiff);
}
void MoveManager::forward_right(float inSpeed, float inSpeedDiff){
forward_right_speed(inSpeed,inSpeedDiff);
}

void MoveManager::forward_right(uint8_t inPowerDiff){
forward_right_power(_cur_turn_power,inPowerDiff);
}
void MoveManager::forward_right(uint8_t inPower, uint8_t inPowerDiff){
forward_right_power(inPower,inPowerDiff);
}

//----------------------------------------------------------------------------
// Move Circle
void MoveManager::circle(){
if(_move_control == MOVE_CONTROL_SPEED){
    circle_speed(_cur_turn_speed,_circle_diff_speed,_circle_direction);
}
else{
    circle_power(_cur_turn_power,_circle_diff_power,_circle_direction);
}
}
void MoveManager::circle(int8_t turnDir){
if(_move_control == MOVE_CONTROL_SPEED){
    circle_speed(_cur_turn_speed,_circle_diff_speed,turnDir);
}
else{
    circle_power(_cur_turn_power,_circle_diff_power,turnDir);
}
}

//============================================================================
// BASIC MOVEMENT FUNCTIONS - CONTROL BY POWER
//============================================================================
// MOVE - POWER CONTROL - SPECIFY POWER
void MoveManager::forward_power(uint8_t inPower){
    _update_basic_move(MOVE_B_FORWARD);
    _motor_left->run(FORWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveManager::back_power(uint8_t inPower){
    _update_basic_move(MOVE_B_BACK);
    _motor_left->run(BACKWARD);
    _motor_right->run(BACKWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveManager::left_power(uint8_t inPower){
    _update_basic_move(MOVE_B_LEFT);
    _motor_left->run(BACKWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveManager::right_power(uint8_t inPower){
    _update_basic_move(MOVE_B_RIGHT);
    _motor_left->run(FORWARD);
    _motor_right->run(BACKWARD);
    _motor_left->setSpeed(inPower);
    _motor_right->setSpeed(inPower);
}

void MoveManager::forward_left_power(uint8_t inPower, uint8_t inPowerDiff){
    _update_basic_move(MOVE_B_FORLEFT);
    _motor_left->run(FORWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower-inPowerDiff/2);
    _motor_right->setSpeed(inPower+inPowerDiff/2);
}

void MoveManager::forward_right_power(uint8_t inPower, uint8_t inPowerDiff){
    _update_basic_move(MOVE_B_FORRIGHT);
    _motor_left->run(FORWARD);
    _motor_right->run(FORWARD);
    _motor_left->setSpeed(inPower+inPowerDiff/2);
    _motor_right->setSpeed(inPower-inPowerDiff/2);
}

void MoveManager::circle_power(uint8_t inPower, int8_t inPowerDiff, int8_t turnDir){
    if(turnDir == MOVE_B_LEFT){
        forward_left_power(inPower,inPowerDiff);
    }
    else{
        forward_right_power(inPower,inPowerDiff);
    }
}

//============================================================================
// BASIC MOVEMENT - SPEED CONTROL with PID
//============================================================================
// MOVE - SPEED CONTROL - SPECIFY SPEED
void MoveManager::forward_speed(float inSpeed){
    _update_basic_move(MOVE_B_FORWARD);
    _at_speed(inSpeed,inSpeed);
}

void MoveManager::back_speed(float inSpeed){
    _update_basic_move(MOVE_B_BACK);
    _at_speed(inSpeed,inSpeed);
}

void MoveManager::left_speed(float inSpeed){
    _update_basic_move(MOVE_B_LEFT);
    _at_speed(-1.0*inSpeed,inSpeed);
}

void MoveManager::forward_left_speed(float inSpeed, float inSpeedDiff){
    _update_basic_move(MOVE_B_FORLEFT);
    _at_speed(inSpeed-(0.5*inSpeedDiff),inSpeed+(0.5*inSpeedDiff));
}

void MoveManager::right_speed(float inSpeed){
    _update_basic_move(MOVE_B_RIGHT);
    _at_speed(inSpeed,-1.0*inSpeed);
}

void MoveManager::forward_right_speed(float inSpeed, float inSpeedDiff){
    _update_basic_move(MOVE_B_FORRIGHT);
    _at_speed(inSpeed+(0.5*inSpeedDiff),inSpeed-(0.5*inSpeedDiff));
}

void MoveManager::circle_speed(float inSpeed, float inSpeedDiff, int8_t turnDir){
    if(turnDir == MOVE_B_LEFT){
        forward_left(inSpeed,inSpeedDiff);
    }
    else{
        forward_right(inSpeed,inSpeedDiff);
    }
}

// NOTE: position can be negative to move backwards
void MoveManager::to_dist_ctrl_pos(float setDist){
    _update_basic_move(MOVE_B_TODIST_CPOS);
    _to_pos(setDist,setDist);
}
void MoveManager::to_dist_ctrl_pos(float setDistL, float setDistR){
    _update_basic_move(MOVE_B_TODIST_CPOS);
    _to_pos(setDistL,setDistR);
}

void MoveManager::turn_to_angle_ctrl_pos(float setAngle){
    _update_basic_move(MOVE_B_TOANG_CPOS);
    float arcLeng = setAngle*_wheel_circ_ang;
    _to_pos(-1.0*arcLeng,arcLeng);
}

int8_t MoveManager::to_dist_ctrl_speed(float speedL, float speedR,
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

        /*
        Serial.print("MMPCount= "); Serial.print(_encoder_left->get_mm_per_count());
        Serial.print(",SetDistL= "); Serial.print(setDistL); Serial.print(",SetDistR= "); Serial.print(setDistR);
        Serial.print(",ECDiffL= "); Serial.print(_encoder_count_diff_left); Serial.print(",ECDiffR= "); Serial.print(_enc_count_diff_right);
        Serial.println();
        Serial.print("StartECount: L="); Serial.print(_start_encoder_count_left); Serial.print(", R="); Serial.print(_start_encoder_count_right);
        Serial.println();
        Serial.print("EndECount: L="); Serial.print(_end_encoder_count_left); Serial.print(", R="); Serial.print(_end_encoder_count_right);
        Serial.println();
        Serial.println();
        */
    }

    if(_timeout_timer.finished()){
        isComplete = 2;
    }
    else{
        if((setDistL > 0.0) && (setDistR > 0.0)){ // Go forward
        if((_encoder_left->get_count() <= _end_encoder_count_left)||(_encoder_right->get_count() <= _end_encoder_count_right)){
            _at_speed(abs(speedL),abs(speedR));
        }
        else{
            isComplete = 1;
            stop_no_update();
        }
        }
        else if((setDistL < 0.0) && (setDistR > 0.0)){ // Turn left
            if((_encoder_left->get_count() >= _end_encoder_count_left)||(_encoder_right->get_count() <= _end_encoder_count_right)){
                _at_speed(-1.0*abs(speedL),abs(speedR));
            }
            else{
                isComplete = 1;
                stop_no_update();
            }
        }
        else if((setDistL > 0.0) && (setDistR < 0.0)){
            if((_encoder_left->get_count() <= _end_encoder_count_left)||(_encoder_right->get_count() >= _end_encoder_count_right)){
                _at_speed(abs(speedL),-1.0*abs(speedR));
            }
            else{
                isComplete = 1;
                stop_no_update();
            }
        }
        else if((setDistL < 0.0) && (setDistR < 0.0)){ // Turn right
            if((_encoder_left->get_count() >= _end_encoder_count_left)||(_encoder_right->get_count() >= _end_encoder_count_right)){
                _at_speed(-1.0*abs(speedL),-1.0*abs(speedR));
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
    if(_move_control == MOVE_CONTROL_SPEED){
        spiral_speed(turnDir);
    }
    else{
        spiral_power(turnDir);
    }
}

void MoveManager::spiral_speed(int8_t turnDir){
    if(_spiral_start || (_move_compound != MOVE_C_SPIRAL)){
        // Reset the flag so we don't re-init,
        _spiral_start = false;
        _move_compound = MOVE_C_SPIRAL;
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
        circle_speed(_cur_forward_speed,_cur_spiral_speed_diff,turnDir);
    }
}

void MoveManager::spiral_power(int8_t turnDir){
    if(_spiral_start || (_move_compound != MOVE_C_SPIRAL)){
        // Reset the flag so we don't re-init,
        _spiral_start = false;
        _move_compound = MOVE_C_SPIRAL;
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
        circle_power(_cur_forward_power,_cur_spiral_speed_diff_power,turnDir);
    }
}

//----------------------------------------------------------------------------
// MOVE ZIG/ZAG
void MoveManager::zig_zag(){
    if(_zz_turn_flag){
        if(!_submove_timer.finished()){
        if(_zz_turn_dir == MOVE_B_LEFT){
            if(_move_control == MOVE_CONTROL_SPEED){
                forward_left(_cur_turn_speed,_zz_turn_diff_speed);
            }
            else{
                forward_left(_cur_turn_power,_zz_turn_diff_power);
            }
        }
        else{
            if(_move_control == MOVE_CONTROL_SPEED){
                forward_right(_cur_turn_speed,_zz_turn_diff_speed);
            }
            else{
                forward_right(_cur_turn_power,_zz_turn_diff_power);
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
        _move_compound = MOVE_C_LOOK;
        _look_start_flag = false;
        _look_timer.start(_look_move_time);
        _look_cur_ang = 0;
        _look_move_switch = true;
        reset_PIDs();
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
    reset_PIDs();
}

void MoveManager::reset_look(){
    //Serial.println("FUNC: reset look.");
    _look_start_flag = true;
    _look_cur_ang = 0;
    reset_PIDs();
}

//----------------------------------------------------------------------------
// PIDs - GET/SET FUNCTIONS
//----------------------------------------------------------------------------
void MoveManager::reset_PIDs(){
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

//----------------------------------------------------------------------------
// PRIVATE HELPER FUNCTIONS
void MoveManager::_update_basic_move(EMoveBasic move){
    if(_move_basic != move){
        _move_basic = move;
        _encoder_count_start = true;
        _start_encoder_count_left = _encoder_left->get_count();
        _start_encoder_count_right = _encoder_right->get_count();
        reset_PIDs();
    }
}

//----------------------------------------------------------------------------
void MoveManager::_update_compound_move(){
    _move_update_time = random(_move_update_min_time,_move_update_max_time);

    if(_move_compound == MOVE_C_SPIRAL){
        _spiral_start = true;
        _spiral_direction = random(0,2);
        _move_update_time = _spiral_duration;
    }
    else if(_move_compound == MOVE_C_CIRCLE){
        _circle_direction = random(0,2);
    }
    else if(_move_compound == MOVE_C_LOOK){
        _look_start_flag = true;
        _move_update_time = _look_tot_time;
    }

    // Restart timers
    _move_timer.start(_move_update_time);
    _submove_timer.start(0);
}

//----------------------------------------------------------------------------
void MoveManager::_at_speed(float inSpeedL,float inSpeedR){
    // Check if the left/right PIDs are on if not turn them on
    if(!_speed_PID_left.get_controller_on()){
        _speed_PID_left.set_controller_on(PID_ON);
        _speed_PID_left.set_Pgain_only(_speed_P_rev);
        if(inSpeedL < 0.0){
        _speed_PID_left.set_controller_dir(PID_REVERSE);

        }
        else{
        _speed_PID_left.set_controller_dir(PID_DIRECT);
        }
        if((inSpeedL < 0.0) && (inSpeedR < 0.0)){
        _speed_PID_right.set_Pgain_only(_speed_P_rev);
        _speed_PID_left.set_Pgain_only(_speed_P_rev);
        }
    }
    if(!_speed_PID_right.get_controller_on()){
        _speed_PID_right.set_controller_on(PID_ON);
        _speed_PID_right.set_Pgain_only(_speed_P_rev);
        if(inSpeedR < 0.0){
        _speed_PID_right.set_controller_dir(PID_REVERSE);
        }
        else{
        _speed_PID_right.set_controller_dir(PID_DIRECT);
        }
        if((inSpeedL < 0.0) && (inSpeedR < 0.0)){
        _speed_PID_right.set_Pgain_only(_speed_P_rev);
        _speed_PID_left.set_Pgain_only(_speed_P_rev);
        }
    }

    // Update the set point
    _speed_PID_left.set_set_point(inSpeedL);
    _speed_PID_right.set_set_point(inSpeedR);

    // Update left and right speed PIDs
    _speed_PID_left.update(_encoder_left->get_smooth_speed_mmps());
    _speed_PID_right.update(_encoder_right->get_smooth_speed_mmps());

    // If the speed is negative then set motors to run backward
    if(inSpeedL < 0.0){
        _motor_left->run(BACKWARD);
    }
    else{
        _motor_left->run(FORWARD);
    }
    if(inSpeedR < 0.0){
        _motor_right->run(BACKWARD);
    }
    else{
        _motor_right->run(FORWARD);
    }
    _motor_left->setSpeed(int(_speed_PID_left.get_output()));
    _motor_right->setSpeed(int(_speed_PID_right.get_output()));
}

//----------------------------------------------------------------------------
void MoveManager::_to_pos(float setPosL, float setPosR){
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
    int32_t checkSetPointL =  round(setPosL/_encoder_left->get_mm_per_count());
    if(checkSetPointL != _set_point_rel_counts_left){
        _set_point_rel_counts_left = checkSetPointL;
        _start_encoder_count_left = _encoder_left->get_count();
        _pos_PID_left.set_set_point(float(_set_point_rel_counts_left));
    }
    // RIGHT
    int32_t checkSetPointR =  round(setPosR/_encoder_right->get_mm_per_count());
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

//----------------------------------------------------------------------------
void MoveManager::_update_speed(){
    _cur_forward_speed = constrain(_def_forward_speed*_speed_mood_fact*_speed_col_fact,_min_speed,_max_speed);
    _cur_back_speed = -1.0*constrain(fabs(_def_back_speed*_speed_mood_fact*_speed_col_fact),_min_speed,_max_speed);
    _cur_turn_speed = constrain(_def_turn_speed*_speed_mood_fact*_speed_col_fact,_min_speed,_max_speed);
}
