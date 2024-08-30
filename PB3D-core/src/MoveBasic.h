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
    void forward_left_diff_frac(float diff_fraction);
    void forward_left_diff_speed(float diff_speed);

    void forward_right();
    void forward_right_diff_frac(float diff_fraction);
    void forward_right_diff_speed(float diff_speed);

    // MOVE - POWER CONTROL - SPECIFY POWER
    void forward_at_power(uint8_t power);
    void back_at_power(uint8_t power);
    void left_at_power(uint8_t power);
    void right_at_power(uint8_t power);
    void forward_left_at_power(uint8_t power, uint8_t power_diff);
    void forward_right_at_power(uint8_t power, uint8_t power_diff);

    // MOVE - SPEED CONTROL - SPECIFY SPEED
    void forward_at_speed(float speed);
    void back_at_speed(float speed);
    void left_at_speed(float speed);
    void right_at_speed(float speed);
    void forward_left_at_speed(float speed, float speed_diff);
    void forward_right_at_speed(float speed, float speed_diff);

    // MOTOR SPEED CONTROL - Get/Set
    float get_forward_speed(){return _forward_speed;}
    float get_back_speed(){return _back_speed;}
    float get_turn_speed(){return _turn_speed;}

    void set_forward_speed(float speed);
    void set_back_speed(float speed);
    void set_turn_speed(float speed);

    void set_speed_base_multiplier(float multiplier);
    void set_speed_mood_multiplier(float multiplier);
    void set_speed_danger_multiplier(EDangerCode danger_code);

    // Motor power variables in pwm counts
    const uint8_t default_forward_power = 120;
    const uint8_t default_back_power = 120;
    const uint8_t default_turn_power = 100;
    const uint8_t default_turn_power_diff = 80;
    const uint8_t min_power = 25;
    const uint8_t max_power = 255;

    // Motor speed variables in mm/s (millimeters per second)
    const float default_forward_speed = 350.0;
    const float default_back_speed = -225.0;
    const float default_turn_speed = 250.0;
    const float default_turn_speed_diff = 0.75*default_turn_speed;
    const float min_speed = 50.0;
    const float max_speed = 1000.0;
    const float speed_danger_true = 0.8;
    const float speed_danger_false = 1.0;

private:
    void _update_speed_with_multipliers();

    EMoveControl _move_control_code = MOVE_CONTROL_POWER;
    Adafruit_MotorShield* _motor_shield = NULL;
    Adafruit_DCMotor* _motor_left = NULL;
    Adafruit_DCMotor* _motor_right = NULL;

    MoveController* _move_controller = NULL;

    uint8_t forward_power = default_forward_power;
    uint8_t back_power = default_back_power;
    uint8_t turn_power = default_turn_power;
    uint8_t turn_power_diff = default_turn_power_diff;

    float _forward_speed = default_forward_speed;
    float _back_speed = default_back_speed;
    float _turn_speed = default_turn_speed;
    float _turn_speed_diff = default_turn_speed_diff;

    float _speed_base_multiplier = 1.0;
    float _speed_mood_multiplier = 1.0;
    float _speed_danger_multiplier = 1.0;
};
#endif
