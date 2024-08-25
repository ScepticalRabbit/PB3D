//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_CONTROLLER_H
#define MOVE_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <PB3DConstants.h>
#include "Encoder.h"
#include "PID.h"

class MoveController{
public:
    MoveController(Adafruit_MotorShield* motor_shield);

    void begin(Encoder* encoder_left, Encoder* encoder_right);

    void at_speed(float speed_left,float speed_right);
    void to_position(float set_pos_left, float set_pos_right);

    void reset();

    PID* get_speed_PID_left(){return &_speed_PID_left;}
    PID* get_speed_PID_right(){return &_speed_PID_right;}
    PID* get_pos_PID_left(){return &_pos_PID_left;}
    PID* get_pos_PID_right(){return &_pos_PID_right;}

private:
    Adafruit_MotorShield* _motor_shield = NULL;
    Adafruit_DCMotor* _motor_left = NULL;
    Adafruit_DCMotor* _motor_right = NULL;
    Encoder* _encoder_left = NULL;
    Encoder* _encoder_right = NULL;

    // TODO: 1st Jan 2023 - need to update gains based on new encoder counts, halved gain as temporary fix (was 0.02)
    //double _speed_P = 2.5, _speed_I = 0.0, _speed_D = 0.0; // NOTE: 2.5 causes osc with 50ms period
    //double _speed_P = 0.45*2.5, _speed_I = 50.0e-3/1.2, _speed_D = 0.0; // Ziegler-Nichols tuning [0.45,/1.2,0.0]
    const double _speed_P = 0.6*0.8;
    const double _speed_I = 0.5*50.0e-3;
    const double _speed_D = 50.0e-3/8.0; // Ziegler-Nichols tuning [0.6,0.5,/8]
    const double _speed_P_rev = _speed_P*0.9; // NOTE: turned off setting gains! Caused problems

    PID _speed_PID_left = PID(false,_speed_P,_speed_I,_speed_D,10);
    PID _speed_PID_right = PID(false,_speed_P,_speed_I,_speed_D,10);

    const uint8_t _speed_PID_min_power = 25.0;

    // Position Control PIDs and Variables
    PID _pos_PID_left = PID(false,0.6,0.0,0.0,20);
    PID _pos_PID_right = PID(false,0.6,0.0,0.0,20);

    const float _pos_PID_min_speed = 100.0;
    const float _pos_PID_max_speed = 200.0;
    int16_t _pos_tol = 3;

    const float _wheel_base = 172.0; // UPDATED: 1st Jan 2023 - new stable geom chassis with large wheels
    const float _wheel_circ = _wheel_base*PI;
    const float _wheel_circ_ang = (_wheel_base*PI)/(360.0); // FIXED factor of 2 by adding encode interrupt
    bool _pos_at_left = false;
    bool _pos_at_right = false;
    bool _pos_at_both = false;
    // NOTE: D(inner) = 122mm, D(outer) = 160mm, D(avg) = 141mm

    int32_t _start_encoder_count_left = 0;
    int32_t _set_point_rel_counts_left = 0;
    int32_t _curr_relative_count_left = 0;
    int32_t _start_encoder_count_right = 0;
    int32_t _set_point_rel_counts_right = 0;
    int32_t _curr_relative_count_right = 0;
};

#endif