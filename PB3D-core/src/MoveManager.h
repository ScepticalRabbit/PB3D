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



class MoveManager{
public:
    MoveManager(Encoder* encoder_left,
                Encoder* encoder_right);

    //----------------------------------------------------------------------------
    // BEGIN: called once during SETUP
    void begin();

    //----------------------------------------------------------------------------
    // UPDATE: Called during LOOP
    void update_move();
    void update_move(int8_t move_type);

    //----------------------------------------------------------------------------
    // GO: Called during explore or other task to randomise movements
    void go();

    //----------------------------------------------------------------------------
    // GET,SET and RESET functions: Inline
    // MOVE TYPE - Get/Set
    int8_t get_basic_move(){return _move_basic_code;}
    int8_t get_compound_move(){return _move_compound_code;}
    void set_compound_move(int8_t move_code){_move_compound_code = move_code;}

    // MOTOR POWER CONTROL - Get/Set
    uint8_t get_forward_power(){return _cur_forward_power;}
    uint8_t get_back_power(){return _cur_back_power;}
    uint8_t get_turn_power(){return _cur_turn_power;}

    void set_forward_power(uint8_t power){_cur_forward_power = power;}
    void set_back_power(uint8_t power){_cur_back_power = power;}
    void set_turn_power(uint8_t power){_cur_turn_power = power;}

    // MOTOR SPEED CONTROL - Get/Set
    float get_min_speed(){return _min_speed;}
    float get_max_speed(){return _max_speed;}

    float get_forward_speed(){return _cur_forward_speed;}
    float get_back_speed(){return _cur_back_speed;}
    float get_turn_speed(){return _cur_turn_speed;}

    void set_forward_speed(float speed){_cur_forward_speed = fabs(speed);}
    void set_back_speed(float speed){_cur_back_speed = -1.0*fabs(speed);}
    void set_turn_speed(float speed){_cur_turn_speed = fabs(speed);}

    // ENCODERS - Get/Set
    int32_t get_encoder_count_left(){return _encoder_left->get_count();}
    int32_t get_encoder_count_right(){return _encoder_right->get_count();}
    float get_encoder_speed_left(){
    return _encoder_left->get_smooth_speed_mmps();}
    float get_encoder_speed_right(){
    return _encoder_right->get_smooth_speed_mmps();}
    float get_encoder_mm_per_count(){return _encoder_left->get_mm_per_count();}

    // MOVE TIMERS - Reset
    void reset_move_timer(){_move_timer.start(0);}
    void reset_submove_timer(){_submove_timer.start(0);}

    //---------------------------------------------------------------------------
    // GET,SET and RESET functions: full implementation
    void set_power_by_diff(int8_t diff);
    void set_speed_by_col_code(bool obstacle_close);
    void set_move_control(int8_t move_control);
    void change_circ_dir();
    void set_speed_by_mood_fact(float moode_fact);

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

    //--------------------------------------------------------------------------
    // Move Forward Left
    void forward_left();
    void forward_left_diff_frac(float diff_frac);

    //--------------------------------------------------------------------------
    // Move Forward Right
    void forward_right();
    void forward_right_diff_frac(float diff_frac);

    //--------------------------------------------------------------------------
    // Controlled movement
    // NOTE: position can be negative to move backwards
    bool get_pos_PID_at_setpoint(){
        return _move_controller.get_pos_PID_both_at_setpoint();}

    void to_dist_ctrl_pos(float set_dist);
    void to_dist_ctrl_pos(float set_dist_left, float set_dist_right);
    void turn_to_angle_ctrl_pos(float set_angle);

    int8_t to_dist_ctrl_speed(float speed_left, float speed_right,
                              float set_dist_left, float set_dist_right);
    int8_t to_dist_ctrl_speed(float set_dist);
    int8_t turn_to_angle_ctrl_speed(float set_angle);

    //--------------------------------------------------------------------------
    // Move Circle
    void circle();
    //void circle(int8_t turn_dir);

    //==========================================================================
    // COMPOUND MOVEMENT FUNCTIONS
    //==========================================================================
    // NOTE: these two functions already work with the default speed
    // ADD: compound move code to each of these

    //--------------------------------------------------------------------------
    // MOVE WIGGLE LEFT/RIGHT
    void wiggle();
    void wiggle(uint16_t left_dur, uint16_t right_dur);

    //--------------------------------------------------------------------------
    // MOVE FORWARD/BACK
    void forward_back();
    void forward_back(uint16_t forward_dur, uint16_t back_dur);

    //--------------------------------------------------------------------------
    // MOVE SPIRAL
    void spiral();
    void spiral(int8_t turn_dir);
    void spiral_speed(int8_t turn_dir);
    void spiral_power(int8_t turn_dir);

    //--------------------------------------------------------------------------
    // MOVE ZIG/ZAG
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
    void _update_compound_move();
    void _update_speed();

    Adafruit_MotorShield _motor_shield = Adafruit_MotorShield(ADDR_MOTOR_SHIELD);
    Adafruit_DCMotor* _motor_left = NULL;
    Adafruit_DCMotor* _motor_right = NULL;

    Encoder* _encoder_left = NULL;
    Encoder* _encoder_right = NULL;

    EMoveControl _move_control_code = MOVE_CONTROL_SPEED;
    MoveController _move_controller = MoveController(&_motor_shield);
    MoveBasic _move_basic = MoveBasic(_move_control_code,&_motor_shield);

    MoveCircle _move_circle = MoveCircle(&_move_basic);


  //----------------------------------------------------------------------------
  // MOVE OBJ - Type and General Variables
  bool _enabled = true;

  int8_t _move_basic_code = MOVE_B_FORWARD;
  int8_t _move_compound_code = MOVE_C_STRAIGHT;
  int8_t _move_compound_count = MOVE_C_COUNT;

  uint32_t _move_update_time = 5000;
  uint32_t _move_update_min_time = 4000;
  uint32_t _move_update_max_time = 12000;

  Timer _move_timer = Timer();
  Timer _submove_timer = Timer();
  Timer _timeout_timer = Timer();

  //----------------------------------------------------------------------------
  // MOVE OBJ - Motor Power (Speed) Variables
  const uint8_t _def_forward_power = 120;
  const uint8_t _def_back_power = 120;
  const uint8_t _def_turn_power = 100;
  const uint8_t _def_turn_power_diff = 80;

  uint8_t _cur_forward_power = _def_forward_power;
  uint8_t _cur_back_power = _def_back_power;
  uint8_t _cur_turn_power = _def_turn_power;
  uint8_t _cur_turn_power_diff = _def_turn_power_diff;

  const uint8_t _min_power = 25;
  const uint8_t _max_power = 255;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Motor Speed Variables in mm/s (millimeters per second)
  const float _def_forward_speed = 350.0;
  const float _def_back_speed = -225.0;
  const float _def_turn_speed = 250.0;
  const float _def_turn_speed_diff = 0.75*_def_turn_speed;

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
  // MOVE OBJ - Circle Variables
  uint8_t _circle_diff_power = 30;
  float _circle_diff_speed = _def_forward_speed*0.5;
  int8_t _circle_direction = MOVE_B_LEFT;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Zig Zag Variables
  bool _zz_turn_flag = true;
  uint16_t _zz_init_turn_dur = 800;
  uint16_t _zz_left_turn_dur = _zz_init_turn_dur;
  uint16_t _zz_right_turn_dur = _zz_init_turn_dur;
  uint16_t _zz_turn_duration = _zz_left_turn_dur;

  bool _zz_straight_flag = false;
  uint32_t _zz_straight_duration = 1000;
  int8_t _zz_turn_dir = MOVE_B_LEFT;

  uint8_t _zz_turn_diff_power = round(_def_forward_power/2);
  float _zz_turn_diff_speed = 0.5*_def_forward_speed;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Spiral Variables
  bool _spiral_start = true;
  uint32_t _spiral_start_time = 0;
  uint32_t _spiral_duration = 20000;
  uint32_t _spiral_curr_time = 0;
  int8_t _spiral_direction = MOVE_B_LEFT;

  float _spiral_min_speed = 5.0;
  float _spiral_slope = 0.0;
  float _init_spiral_speed_diff = _def_forward_speed-_min_speed;
  float _cur_spiral_speed_diff = 0.0;

  uint8_t _spiral_min_power = 5.0;
  float _spiral_slope_power = 0.0;
  uint8_t _init_spiral_speed_diff_power = _def_forward_power-_min_power;
  uint8_t _cur_spiral_speed_diff_power = 0;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Wiggle Variables
  bool _wiggle_left_flag = true;
  uint16_t _wiggle_def_left_dur = 600;
  uint16_t _wiggle_def_right_dur = 600;
  uint16_t _wiggle_curr_dur = _wiggle_def_left_dur;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Forward/Back Variables
  bool _FB_forward_flag = true;
  uint16_t _FB_def_forward_dur = 500;
  uint16_t _FB_def_back_dur = 500;
  uint16_t _FB_curr_dur = _FB_def_forward_dur;

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
