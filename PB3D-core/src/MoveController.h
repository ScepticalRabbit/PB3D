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
#include "WheelData.h"

class MoveController{
public:
    MoveController(Adafruit_MotorShield* motor_shield);

    void begin(Encoder* encoder_left, Encoder* encoder_right);

    EMoveControlState at_speed(float speed_left,float speed_right);
    EMoveControlState to_position(float set_pos_left, float set_pos_right);

    EMoveControlState to_dist_ctrl_pos(float set_dist_left,
                                       float set_dist_right);
    EMoveControlState turn_to_angle_ctrl_pos(float set_angle);

    void reset();

    EMoveControlState get_speed_PID_state(){return _speed_ctrl_state;}
    PID* get_speed_PID_left(){return &_speed_PID_left;}
    PID* get_speed_PID_right(){return &_speed_PID_right;}

    EMoveControlState get_pos_PID_state(){return _pos_ctrl_state;}
    PID* get_pos_PID_left(){return &_pos_PID_left;}
    PID* get_pos_PID_right(){return &_pos_PID_right;}

private:
    Adafruit_MotorShield* _motor_shield = NULL;
    Adafruit_DCMotor* _motor_left = NULL;
    Adafruit_DCMotor* _motor_right = NULL;
    Encoder* _encoder_left = NULL;
    Encoder* _encoder_right = NULL;

    WheelData wheel_data = WheelData();

    // TODO: 1st Jan 2023 - need to update gains based on new encoder counts, halved gain as temporary fix (was 0.02)
    //double _speed_P = 2.5, _speed_I = 0.0, _speed_D = 0.0; // NOTE: 2.5 causes osc with 50ms period
    //double _speed_P = 0.45*2.5, _speed_I = 50.0e-3/1.2, _speed_D = 0.0; // Ziegler-Nichols tuning [0.45,/1.2,0.0]
    const double _speed_P = 0.6*0.8;
    const double _speed_I = 0.5*50.0e-3;
    const double _speed_D = 50.0e-3/8.0; // Ziegler-Nichols tuning [0.6,0.5,/8]
    const double _speed_P_rev = _speed_P*0.9; // NOTE: turned off setting gains! Caused problems

    EMoveControlState _speed_ctrl_state = MOVE_CONTROL_INCOMPLETE;
    PID _speed_PID_left = PID(false,_speed_P,_speed_I,_speed_D,10);
    PID _speed_PID_right = PID(false,_speed_P,_speed_I,_speed_D,10);

    const uint8_t _speed_PID_min_power = 25.0;
    const float _speed_tol = 25.0;

    // Position Control PIDs and Variables
    EMoveControlState _pos_ctrl_state = MOVE_CONTROL_INCOMPLETE;
    PID _pos_PID_left = PID(false,0.6,0.0,0.0,20);
    PID _pos_PID_right = PID(false,0.6,0.0,0.0,20);

    const float _pos_PID_min_speed = 100.0;
    const float _pos_PID_max_speed = 200.0;
    const int16_t _pos_tol = 3;

    int32_t _start_encoder_count_left = 0;
    int32_t _set_point_rel_counts_left = 0;
    int32_t _relative_count_left = 0;

    int32_t _start_encoder_count_right = 0;
    int32_t _set_point_rel_counts_right = 0;
    int32_t _relative_count_right = 0;
};

#endif