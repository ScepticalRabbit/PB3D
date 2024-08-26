//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_BASIC_H
#define MOVE_BASIC_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <PB3DConstants.h>
#include "MoveController.h"

class MoveBasic{
public:
    MoveBasic(EMoveControl move_control,
              Adafruit_MotorShield* motor_shield){
        _move_control_code = move_control;
        _motor_shield = motor_shield;
    }

    void begin(MoveController* move_controller);

    void update_basic_move(EMoveBasic move);

    void stop();
    void forward();
    void back();
    void left();
    void right();

    void forward_left();
    void forward_left_diff_frac(float diff_frac);
    void forward_left(float speed_diff);
    void forward_left(float speed, float speed_diff);
    void forward_left(uint8_t power_diff);
    void forward_left(uint8_t power, uint8_t power_diff);

    void forward_right();
    void forward_right_diff_frac(float diff_frac);
    void forward_right(float speed_diff);
    void forward_right(float speed, float speed_diff);
    void forward_right(uint8_t power_diff);
    void forward_right(uint8_t power, uint8_t power_diff);

         // MOVE - POWER CONTROL - SPECIFY POWER
    void forward_power(uint8_t power);
    void back_power(uint8_t power);
    void left_power(uint8_t power);
    void right_power(uint8_t power);
    void forward_left_power(uint8_t power, uint8_t power_diff);
    void forward_right_power(uint8_t power, uint8_t power_diff);

    // MOVE - SPEED CONTROL - SPECIFY SPEED
    void forward_speed(float speed);
    void back_speed(float speed);
    void left_speed(float speed);
    void forward_left_speed(float speed, float speed_diff);
    void right_speed(float speed);
    void forward_right_speed(float speed, float speed_diff);

private:
    EMoveControl _move_control_code = MOVE_CONTROL_POWER;
    Adafruit_MotorShield* _motor_shield = NULL;
    Adafruit_DCMotor* _motor_left = NULL;
    Adafruit_DCMotor* _motor_right = NULL;

    MoveController* _move_controller = NULL;

    // MOVE Motor Power (Speed) Variables
    uint8_t _def_forward_power = 120;
    uint8_t _def_back_power = 120;
    uint8_t _def_turn_power = 100;
    uint8_t _def_turn_power_diff = 80;

    uint8_t _cur_forward_power = _def_forward_power;
    uint8_t _cur_back_power = _def_back_power;
    uint8_t _cur_turn_power = _def_turn_power;
    uint8_t _cur_turn_power_diff = _def_turn_power_diff;

    uint8_t _min_power = 25;
    uint8_t _max_power = 255;

    // MOVE OBJ - Motor Speed Variables in mm/s (millimeters per second)
    float _def_forward_speed = 350.0;
    float _def_back_speed = -225.0;
    float _def_turn_speed = 250.0;
    float _def_turn_speed_diff = 0.75*_def_turn_speed;

    float _cur_forward_speed = _def_forward_speed;
    float _cur_back_speed = _def_back_speed;
    float _cur_turn_speed = _def_turn_speed;
    float _curTurnSpeedDiff = _def_turn_speed_diff;

    float _speed_mood_fact = 1.0;
    float _speed_col_fact = 1.0;
    float _speed_col_true = 0.8;
    float _speed_col_false = 1.0;
    const float _min_speed = 50.0;
    const float _max_speed = 1000.0;
};
#endif
