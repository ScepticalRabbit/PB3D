//==============================================================================
// PB3D: A pet robot that is 3D printed
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
    // MOVE TYPE - Get/Set
    EMoveBasic get_basic_move(){return _move_basic_code;}
    EMoveCompound get_compound_move(){return _move_compound_code;}
    void set_compound_move(EMoveCompound move_code){_move_compound_code = move_code;}

    // MOTOR SPEED CONTROL - Get/Set
    float get_forward_speed();
    float get_back_speed();
    float get_turn_speed();

    void set_speed_base_multiplier(float multiplier);
    void set_speed_mood_multiplier(float multiplier);
    void set_speed_danger_multiplier(EDangerCode danger_code);

    void change_turn_dir();

    // ENCODERS - Get, allows forwarding through move manager
    // NOTE: encoders must be in main for interrupts
    Encoder* get_encoder_left(){return _encoder_left;}
    Encoder* get_encoder_right(){return _encoder_left;}

    //--------------------------------------------------------------------------
    // CALCULATORS
    uint16_t calc_timeout(float speed, float dist);

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
    bool get_pos_controller_at_setpoint(){
        return _move_controller.get_pos_PID_both_at_setpoint();}

    void to_dist_ctrl_pos(float set_dist);
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

    //--------------------------------------------------------------------------
    // MOVE LOOK AROUND
    void look_around();
    void force_look_move();
    void reset_look();

    // LOOK - Diagnostics
    bool look_is_moving(){return _look_move_switch;}
    bool look_is_paused(){return !_look_move_switch;}

    // LOOK - Getters
    bool get_look_move_switch(){return _look_move_switch;}
    bool get_look_finished(){return (_look_cur_ang >= _look_num_angs);}
    uint8_t get_look_curr_ang_ind(){return _look_cur_ang;}
    uint8_t get_look_num_angs(){return _look_num_angs;}
    uint8_t get_look_max_angs(){return _look_max_angs;}
    float get_look_ang_from_ind(uint8_t index){return _look_angles[index];}
    uint16_t get_look_move_time(){return _look_move_time;}
    uint16_t get_look_pause_time(){return _look_pause_time;}
    uint16_t get_look_tot_time(){return _look_tot_time;}

private:
    void _update_basic_move(EMoveBasic move);
    void _update_compound_move(EMoveCompound move);

    Adafruit_MotorShield _motor_shield = Adafruit_MotorShield(ADDR_MOTOR_SHIELD);
    Adafruit_DCMotor* _motor_left = NULL;
    Adafruit_DCMotor* _motor_right = NULL;

    Encoder* _encoder_left = NULL;
    Encoder* _encoder_right = NULL;

    EMoveControlMode _move_control_code = MOVE_CONTROL_SPEED;
    MoveController _move_controller = MoveController(&_motor_shield);
    MoveBasic _move_basic = MoveBasic(_move_control_code,&_motor_shield);

    MoveCircle _move_circle = MoveCircle(&_move_basic);
    MoveWiggle _move_wiggle = MoveWiggle(&_move_basic,&_submove_timer);
    MoveForwardBack _move_forward_back =
            MoveForwardBack(&_move_basic,&_submove_timer);
    MoveSpiral _move_spiral = MoveSpiral(&_move_basic,&_submove_timer);
    MoveZigZag _mov_zig_zag = MoveZigZag(&_move_basic,&_submove_timer);

    //----------------------------------------------------------------------------
    // MOVE OBJ - Type and General Variables
    bool _enabled = true;

    EMoveBasic _move_basic_code = MOVE_B_FORWARD;
    EMoveCompound _move_compound_code = MOVE_C_STRAIGHT;
    EMoveCompound _move_compound_count = MOVE_C_COUNT;

    uint32_t _move_update_time = 5000;
    uint32_t _move_update_min_time = 4000;
    uint32_t _move_update_max_time = 12000;

    Timer _move_timer = Timer();
    Timer _submove_timer = Timer();
    Timer _timeout_timer = Timer();

  // Estimating power for given speed - updated for new wheels - 24th Sept 2022
  // NOTE: turned speed estimation off because PID has less overshoot without
  // Updated again 5th Jan 2023 - RF wood floor tests - slope=0.166,int=22.7
  // Used to be set with an offset of 50.0 and slope 0.0
  float _speed_to_power_slope = 0.166;
  float _speed_to_power_offset = 22.7;
  float _speed_to_power_min = 22.7;
  float _speed_timeout_accel = 1220.0;
  float _speed_timeout_SF = 2.0;

  // Wheel base constant parameters
  // NOTE: D(inner) = 122mm, D(outer) = 160mm, D(avg) = 141mm
  const float _wheel_base = 172.0; // UPDATED: 1st Jan 2023 - new stable geom chassis with large wheels
  const float _wheel_circ = _wheel_base*PI;
  const float _wheel_circ_ang = (_wheel_base*PI)/(360.0); // FIXED factor of 2 by adding encoder interrupt

  //----------------------------------------------------------------------------
  // MOVE OBJ - To Dist/Angle
  float _to_dist_set_pt_left = 0.0;
  float _to_dist_set_pt_right = 0.0;
  float _to_dist_tol = 5.0;
  float _to_ang_set_pt = 0.0;
  float _to_ang_tol = 1.0;

  // Encoder counter variables - Used for PID controlled movement functions
  bool _encoder_count_start = true;
  int32_t _start_encoder_count_left = 0;
  int32_t _start_encoder_count_right = 0;
  int32_t _end_encoder_count_left = 0;
  int32_t _end_encoder_count_right = 0;
  int32_t _encoder_count_diff_left = 0;
  int32_t _enc_count_diff_right = 0;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Look Around
  bool _look_start_flag = true;
  Timer _look_timer = Timer();
  uint16_t _look_move_time = 2200;
  uint16_t _look_pause_time = 500;
  bool _look_move_switch = false;
  uint8_t _look_cur_ang = 0;
  uint8_t _look_num_angs = 4;
  uint8_t _look_max_angs = 8;
  float _look_angles[8] = {30,0,-30,0,0,0,0,0};
  uint16_t _look_tot_time = _look_num_angs*(_look_move_time+_look_pause_time);

};
#endif // MOVE_H
