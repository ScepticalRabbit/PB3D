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
    _look_timer.start(0);
    _timeout_timer.start(0);
  }

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void MoveManager::update(){
    if(!_enabled){return;}

    if(_move_timer.finished()){
        EMoveCompound move = EMoveCompound(
                             random(0,_move_compound_count));
        _update_compound_move(move);
    }
}

void MoveManager::update(EMoveCompound move){
    if(!_enabled){return;}

    if(_move_timer.finished()){
        _update_compound_move(move);
    }
}

void MoveManager::force_update(EMoveCompound move){
    if(!_enabled){return;}

    _update_compound_move(move);
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

void MoveManager::set_speed_by_col_code(EDangerCode obstacle_close){
    _move_basic.set_speed_danger_multiplier(obstacle_close);
}

void MoveManager::set_speed_by_mood_fact(float in_fact){
    _move_basic.set_speed_mood_multiplier(in_fact);
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


//----------------------------------------------------------------------------
// Move Circle


//============================================================================
// ENCODER/PID CONTROLLED MOVEMENT FUNCTIONS
//============================================================================

// NOTE: position can be negative to move backwards
void MoveManager::to_dist_ctrl_pos(float set_dist){
    _update_basic_move(MOVE_B_TODIST_CPOS);
    _move_controller.to_position(set_dist,set_dist);
}

void MoveManager::to_dist_ctrl_pos(float set_dist_left, float set_dist_right){
    _update_basic_move(MOVE_B_TODIST_CPOS);
    _move_controller.to_position(set_dist_left,set_dist_right);
}

void MoveManager::turn_to_angle_ctrl_pos(float set_angle){
    _update_basic_move(MOVE_B_TOANG_CPOS);
    float arcLeng = set_angle*_wheel_circ_ang;
    _move_controller.to_position(-1.0*arcLeng,arcLeng);
}

int8_t MoveManager::to_dist_ctrl_speed(float speed_left, float speed_right,
                                       float set_dist_left, float set_dist_right){

    _update_basic_move(MOVE_B_TODIST_CSpeed);
    int8_t isComplete = 0;

    // If the set distance changes outside tolerance force update
    if(!((set_dist_left >= (_to_dist_set_pt_left-_to_dist_tol)) && (set_dist_left <= (_to_dist_set_pt_left+_to_dist_tol)))){
        _to_dist_set_pt_left = set_dist_left;
        _update_basic_move(MOVE_B_FORCEUPD);
    }
    if(!((set_dist_right >= (_to_dist_set_pt_right-_to_dist_tol)) && (set_dist_right <= (_to_dist_set_pt_right+_to_dist_tol)))){
        _to_dist_set_pt_right = set_dist_right;
        _update_basic_move(MOVE_B_FORCEUPD);
    }

    // At the start we store our target counts for each encode
    if(_encoder_count_start){
        uint16_t timeoutL = calc_timeout(speed_left,set_dist_left);
        uint16_t timeoutR = calc_timeout(speed_right,set_dist_right);
        if(timeoutL > timeoutR){
        _timeout_timer.start(timeoutL);
        }
        else{
        _timeout_timer.start(timeoutR);
        }

        _encoder_count_start = false;
        _encoder_count_diff_left = int32_t(set_dist_left/_encoder_left->get_mm_per_count());
        _enc_count_diff_right = int32_t(set_dist_right/_encoder_right->get_mm_per_count());
        _end_encoder_count_left = _start_encoder_count_left + _encoder_count_diff_left;
        _end_encoder_count_right = _start_encoder_count_right + _enc_count_diff_right;


        Serial.print("MMPCount= "); Serial.print(_encoder_left->get_mm_per_count());
        Serial.print(",SetDistL= "); Serial.print(set_dist_left); Serial.print(",SetDistR= "); Serial.print(set_dist_right);
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
        if((set_dist_left > 0.0) && (set_dist_right > 0.0)){ // Go forward
        if((_encoder_left->get_count() <= _end_encoder_count_left)||(_encoder_right->get_count() <= _end_encoder_count_right)){
            _move_controller.at_speed(abs(speed_left),abs(speed_right));
        }
        else{
            isComplete = 1;
            stop_no_update();
        }
        }
        else if((set_dist_left < 0.0) && (set_dist_right > 0.0)){ // Turn left
            if((_encoder_left->get_count() >= _end_encoder_count_left)||(_encoder_right->get_count() <= _end_encoder_count_right)){
                _move_controller.at_speed(-1.0*abs(speed_left),abs(speed_right));
            }
            else{
                isComplete = 1;
                stop_no_update();
            }
        }
        else if((set_dist_left > 0.0) && (set_dist_right < 0.0)){
            if((_encoder_left->get_count() <= _end_encoder_count_left)||(_encoder_right->get_count() >= _end_encoder_count_right)){
                _move_controller.at_speed(abs(speed_left),-1.0*abs(speed_right));
            }
            else{
                isComplete = 1;
                stop_no_update();
            }
        }
        else if((set_dist_left < 0.0) && (set_dist_right < 0.0)){ // Turn right
            if((_encoder_left->get_count() >= _end_encoder_count_left)||(_encoder_right->get_count() >= _end_encoder_count_right)){
                _move_controller.at_speed(-1.0*abs(speed_left),-1.0*abs(speed_right));
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

int8_t MoveManager::to_dist_ctrl_speed(float set_dist){
    int8_t isComplete = 0;
    if(set_dist < 0.0){
        isComplete = to_dist_ctrl_speed(back_speed,back_speed,set_dist,set_dist);
    }
    else{
        isComplete = to_dist_ctrl_speed(forward_speed,forward_speed,set_dist,set_dist);
    }
    return isComplete;
}

int8_t MoveManager::turn_to_angle_ctrl_speed(float set_angle){
    float set_dist = set_angle*_wheel_circ_ang;
    int8_t isComplete = 0;
    if(set_angle > 0.0){ // Turn left
        isComplete = to_dist_ctrl_speed(-1.0*turn_speed,turn_speed,-1.0*set_dist,set_dist);
    }
    else if(set_angle < 0.0){
        isComplete = to_dist_ctrl_speed(turn_speed,-1.0*turn_speed,-1.0*set_dist,set_dist);
    }
    else{
        isComplete = 1;
    }
    return isComplete;
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
    _move_spiral.spiral(turn_dir);
}

void MoveManager::wiggle(){
    _move_compound_code = MOVE_C_WIGGLE;
    _move_wiggle.wiggle();
}

void MoveManager::wiggle(uint16_t left_time, uint16_t right_time){
    _move_compound_code = MOVE_C_WIGGLE;
    _move_wiggle.wiggle(left_time,right_time);
}

void MoveManager::zig_zag(){
    _mov_zig_zag.zig_zag();
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
// Update helper functions to reset movements
void MoveManager::_update_basic_move(EMoveBasic move){
    if(_move_basic_code != move){
        _move_basic_code = move;
        _encoder_count_start = true;
        _start_encoder_count_left = _encoder_left->get_count();
        _start_encoder_count_right = _encoder_right->get_count();
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
            _look_start_flag = true;
            _move_update_time = _look_tot_time;
        }

        _move_timer.start(_move_update_time);
        _submove_timer.start(0);
    }
}

