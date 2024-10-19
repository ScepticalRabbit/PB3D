//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_H
#define MOVE_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <PB3DI2CAddresses.h>
#include <PB3DConstants.h>
#include <PB3DTimer.h>
#include "Encoder.h"
#include "PID.h"
#include "MoveController.h"
#include "MoveBasic.h"

#include "MoveCircle.h"
#include "MoveForwardBack.h"
#include "MoveLook.h"
#include "MoveSpiral.h"
#include "MoveWiggle.h"
#include "MoveZigZag.h"


class MoveManager{
public:
    MoveManager(Encoder* encoder_left,
                Encoder* encoder_right);

    //----------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin();

    //----------------------------------------------------------------------------
    // UPDATE: Called during LOOP
    void update();
    void update(EMoveCompound move_type);
    void force_update(EMoveCompound move_type);

    //----------------------------------------------------------------------------
    // GO: Called during explore or other task to randomise movements
    void go();

    //----------------------------------------------------------------------------
    // GET,SET and RESET functions
    EMoveBasic get_basic_move(){return _move_basic_code;}
    EMoveCompound get_compound_move(){return _move_compound_code;}
    void set_compound_move(EMoveCompound move_code){_move_compound_code = move_code;}

    float get_forward_speed();
    float get_back_speed();
    float get_turn_speed();

    // Mainly used for debugging PID
    void set_forward_speed(float speed);
    void set_back_speed(float speed);
    void set_turn_speed(float speed);

    void set_speed_base_multiplier(float multiplier);
    void set_speed_mood_multiplier(float multiplier);
    void set_speed_danger_multiplier(EDangerCode danger_code);

    void change_turn_dir();

    // ENCODERS - Get, allows forwarding through move manager
    // NOTE: encoders must be in main for interrupts
    Encoder* get_encoder_left(){return _encoder_left;}
    Encoder* get_encoder_right(){return _encoder_left;}

    //--------------------------------------------------------------------------
    // BASIC MOVEMENT FUNCTIONS - GENERIC (switched by _move_control_code var)
    void stop();
    void stop_no_update();

    void forward();
    void back();
    void left();
    void right();

    void forward_left();
    void forward_left_diff_frac(float diff_frac);
    void forward_left_diff_speed(float diff_speed);

    void forward_right();
    void forward_right_diff_frac(float diff_frac);
    void forward_right_diff_speed(float diff_speed);

    //--------------------------------------------------------------------------
    // Controlled movement
    // NOTE: position can be negative to move backwards

    EMoveControlState get_pos_controller_state(){
        return _move_controller.get_pos_PID_state();}

    void to_dist_ctrl_pos(float set_dist_left, float set_dist_right);
    void turn_to_angle_ctrl_pos(float set_angle);

    EMoveControlState to_dist_ctrl_speed(float speed_left, float speed_right,
                                    float set_dist_left, float set_dist_right);
    EMoveControlState to_dist_ctrl_speed(float set_dist);
    EMoveControlState turn_to_angle_ctrl_speed(float set_angle);


    //==========================================================================
    // COMPOUND MOVEMENT FUNCTIONS
    //==========================================================================
    void circle();

    void forward_back();
    void forward_back(uint16_t forward_time, uint16_t back_time);

    void wiggle();
    void wiggle(uint16_t left_time, uint16_t right_time);

    void spiral();
    void spiral(EMoveTurn turn_dir);

    void zig_zag();

    EMoveLookState look_around();
    void force_look_move();
    void reset_look();
    EMoveLookState get_look_state(){return _move_look.get_look_state();}
    uint8_t get_look_angle_index(){return _move_look.get_look_angle_index();}

    // Turn on/off the movement
    bool enabled = true;

    //==========================================================================
    // Get sub objects
    MoveController* get_move_controller(){return &_move_controller;}
    MoveBasic* get_move_basic(){return &_move_basic;}

private:
    void _update_basic_move(EMoveBasic move);
    void _update_compound_move(EMoveCompound move);

    Adafruit_MotorShield _motor_shield = Adafruit_MotorShield(ADDR_MOTOR_SHIELD);
    Adafruit_DCMotor* _motor_left = NULL;
    Adafruit_DCMotor* _motor_right = NULL;

    Encoder* _encoder_left = NULL;
    Encoder* _encoder_right = NULL;

    EMoveBasic _move_basic_code = MOVE_B_FORWARD;
    EMoveCompound _move_compound_code = MOVE_C_STRAIGHT;
    EMoveCompound _move_compound_count = MOVE_C_COUNT;

    EMoveControlMode _move_control_code = MOVE_MODE_SPEED;
    MoveController _move_controller = MoveController(&_motor_shield);
    MoveBasic _move_basic = MoveBasic(_move_control_code,&_motor_shield);

    // Compound movements using basic/controller movements
    MoveCircle _move_circle = MoveCircle(&_move_basic);
    MoveLook _move_look = MoveLook(&_move_basic,&_move_controller);
    MoveForwardBack _move_forward_back =
            MoveForwardBack(&_move_basic,&_submove_timer);
    MoveSpiral _move_spiral = MoveSpiral(&_move_basic,&_submove_timer);
    MoveWiggle _move_wiggle = MoveWiggle(&_move_basic,&_submove_timer);
    MoveZigZag _mov_zig_zag = MoveZigZag(&_move_basic,&_submove_timer);

    WheelData wheel_data = WheelData();

    uint32_t _move_update_time = 5000;
    const uint32_t _move_update_min_time = 4000;
    const uint32_t _move_update_max_time = 12000;

    Timer _move_timer = Timer();
    Timer _submove_timer = Timer();
};
#endif // MOVE_MANAGER_H
