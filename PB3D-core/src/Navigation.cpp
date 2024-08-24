//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "Navigation.h"

Navigation::Navigation(Encoder* encoder_left,
                       Encoder* encoder_right,
                       IMUSensor* IMU){
    _encoder_left = encoder_left;
    _encoder_right = encoder_right;
    _IMU = IMU;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void Navigation::begin(){
    if(!_IMU->get_enabled_flag()){
        Serial.println(F("NAV: IMU disabled, navigation disabled."));
    }
    else{
        Serial.println(F("NAV: IMU found, navigation enabled."));
    }

    _nav_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void Navigation::update(){
    if(!_enabled){return;}

    if(_nav_timer.finished()){
        _nav_timer.start(_nav_update_time);

        // Store the last calculated coords as previous
        _pos_x_prev = _pos_x_next;
        _pos_y_prev = _pos_y_next;

        // Get the speed of the robots centroid and current heading
        _vel_c = (_encoder_left->get_smooth_speed_mmps()+
                    _encoder_right->get_smooth_speed_mmps()) / 2.0;
        _heading = _IMU->get_head_angle();

        // Calculate the velocity components (X points to front face of robot)
        _vel_x = _vel_c*cos((_heading/180.0)*float(PI));
        _vel_y = _vel_c*sin((_heading/180.0)*float(PI));

        // Update the predicted position
        _pos_x_next = _pos_x_prev + _vel_x*_dt;
        _pos_y_next = _pos_y_prev + _vel_y*_dt;

        // Debug prints update every X interations based on debugFreq
        if(_debug_count++ >= _debug_freq){
        _debug_count = 0; // Reset the debug count

        #if defined(NAV_DEBUG_TIMER)
            Serial.print(F("NAV: update took "));
            Serial.print(_nav_timer.get_time());
            Serial.println(F("ms"));
        #endif

        #if defined(NAV_DEBUG_POS)
            Serial.print(F("NAV: Pos X= "));
            Serial.print(_pos_x_next);
            Serial.print(F("mm, Pos Y= "));
            Serial.print(_pos_y_next);
            Serial.println(F("mm"));
        #endif
        }
    }
}