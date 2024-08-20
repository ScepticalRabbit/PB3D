//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "Tail.h"

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void Tail::begin(){
    _tail_servo.attach(TAIL_SERVO_POUT);
    _wag_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void Tail::update(){
    if(!_enabled){return;}

    if(_update_timer.finished()){
        _update_timer.start(_update_interval);

        switch(_curr_state){
        case TAIL_SET_POS:
            _tail_servo.write(_tail_pos_curr);
            break;
        case TAIL_WAG_CON:
            wag_continuous();
            break;
        case TAIL_WAG_INT:
            wag_interval();
            break;
        default:
            _tail_servo.write(_tail_pos_cent);
            break;
        }
    }
}

//---------------------------------------------------------------------------
// TAIL FUNCTIONS
void Tail::wag_continuous(){
    if(_wag_timer.finished()){
        _wag_timer.start(_wag_move_time);

        if(_wag_switch){
        _wag_switch = !_wag_switch;
        _tail_servo.write(_tail_pos_cent-_wag_pos_offset);
        }
        else{
        _wag_switch = !_wag_switch;
        _tail_servo.write(_tail_pos_cent+_wag_pos_offset);
        }
    }
}

void Tail::wag_interval(){
    if(_wag_timer.finished()){
        if(_wag_count<_wag_count_limit){
        _wag_timer.start(_wag_move_time);

        if(_wag_switch){
            _wag_switch = !_wag_switch;
            _tail_servo.write(_tail_pos_cent-_wag_pos_offset);
        }
        else{
            _wag_switch = !_wag_switch;
            _tail_servo.write(_tail_pos_cent+_wag_pos_offset);
            _wag_count++;
        }
        }
        else{
        _wag_timer.start(_wag_pause_time);
        _tail_servo.write(_tail_pos_cent);
        _wag_count = 0;
        }
    }
}

void Tail::set_wag_params(uint16_t move_time,
                    int16_t offset,
                    uint16_t pause_time,
                    uint8_t count_limit){
    _wag_move_time = move_time;
    _wag_pos_offset = offset;
    _wag_pause_time = pause_time;
    _wag_count_limit = count_limit;
}

void Tail::set_wag_con_params(uint16_t move_time, int16_t offset){
    _wag_move_time = move_time;
    _wag_pos_offset = offset;
}

void Tail::set_wag_int_params(uint16_t move_time,
                        int16_t offset,
                        uint16_t pause_time,
                        uint8_t count_limit){
    _wag_move_time = move_time;
    _wag_pos_offset = offset;
    _wag_pause_time = pause_time;
    _wag_count_limit = count_limit;
}

void Tail::reset(){
    _curr_state = TAIL_CENT;
    _wag_count = 0;
}

