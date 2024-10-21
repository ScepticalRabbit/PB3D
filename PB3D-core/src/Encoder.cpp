//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "Encoder.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR
Encoder::Encoder(int8_t pinA, int8_t pinB){
    _pin_a = pinA;
    _pin_b = pinB;
    _current_count = 0;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void Encoder::begin(){
    pinMode(_pin_a, INPUT);
    pinMode(_pin_b, INPUT);
    _speed_timer.start(0);
    _speed_filt_mmps.begin();
    _speed_filt_cps.begin();
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
// Update for the right encoder '!='
void Encoder::update_not_equal(){
    if(digitalRead(_pin_a) != digitalRead(_pin_b)){
        _current_count++;
        _direction = ENCODER_FORWARD;
    }
    else{
        _current_count--;
        _direction = ENCODER_BACK;
    }
}

// Update for the left encoder '=='
void Encoder::update_equal(){
    if(digitalRead(_pin_a) == digitalRead(_pin_b)){
        _current_count++;
        _direction = ENCODER_FORWARD;
    }
    else{
        _current_count--;
        _direction = ENCODER_BACK;
    }
}

//-------------------------------------------------------------------------
// SPEED CALCULATION FUNCTIONS
void Encoder::update_speed(){
    if(_speed_timer.finished()){
        double timeIntS = double(_speed_timer.get_time())/1000.0;
        _speed_timer.start(_speed_update_time);

         double distMM = double(_current_count-_count_prev_for_speed)*_mm_per_count;

        _raw_speed_cps = double(_current_count-_count_prev_for_speed)/timeIntS;
         _smooth_speed_cps = _speed_filt_cps.filter(_raw_speed_cps);

        _raw_speed_mmps = distMM/timeIntS;
        _smooth_speed_mmps = _speed_filt_mmps.filter(_raw_speed_mmps);

        _count_prev_for_speed = _current_count;
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void Encoder::reset_filter(){
    _speed_filt_mmps.reset();
    _speed_filt_cps.reset();
}

void Encoder::reset_filter(float val){
    _speed_filt_mmps.reset(val);
    _speed_filt_cps.reset(val);
}