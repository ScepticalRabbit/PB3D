//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef I2CDATASENDER_H
#define I2CDATASENDER_H

#include <Arduino.h>
#include <Wire.h>

#include <PB3DI2CAddresses.h>
#include <PB3DStateData.h>

#include "CollisionManager.h"
#include "PB3DTimer.h"
#include "IMUSensor.h"
#include "Navigation.h"


// Debug flags
//#define I2CDATASENDER_DEBUG_PRINT

//----------------------------------------------------------------------------
// CLASS: I2CDataSender
class I2CDataSender{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR: pass in pointers to main objects and other sensors
  I2CDataSender(CollisionManager* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
              IMUSensor* inIMU, Navigation* inNav){
      _collision_manager = inCollision;
      _mood_manager = inMood;
      _task_manager = inTask;
      _move_manager = inMove;
      _IMU = inIMU;
      _navigator = inNav;
  }


  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin(){
    _init_state_data(&_curr_state);
    _update_state_data(&_curr_state);
    _I2C_timer.start(_I2C_time);

    #if defined(I2CDATASENDER_DEBUG_PRINT)
        Serial.println(F("INITIAL DATA STRUCT"));
        _print_state_data(&_curr_state);
    #endif

    _last_col = _collision_manager->get_last_collision();
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update(){
    if(!_enabled){return;}

    if(_I2C_timer.finished()){
        _I2C_timer.start(_I2C_time);
        _send_timer.start(0);

        _update_state_data(&_curr_state);

        Wire.beginTransmission(ADDR_FOLLOW_XIAO_1);
        Wire.write(_curr_state.data_packet,PACKET_SIZE);
        Wire.endTransmission();

        #if defined(I2CDATASENDER_DEBUG_PRINT)
            Serial.print(F("I2C Send Time: "));
            Serial.print(_send_timer.get_time());
            Serial.println(F("ms"));

            Serial.println(F("SENT DATA STRUCTURE:"));
            _print_state_data(&_curr_state);
        #endif
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _enabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_enabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS

  // UPDATE: only used by this class so not in the header
  void _update_state_data(UDataPacket* in_state){

    #if defined(STATEDATA_LASTCOL)
        in_state->state.on_time = millis();

        for(uint8_t ii=0 ; ii<BUMP_COUNT ; ii++){
            in_state->state.check_bumpers[ii] = _last_col->check_bumpers[ii];
        }

        for(uint8_t ii=0 ; ii<LASER_COUNT ; ii++){
            in_state->state.check_lasers[ii] =
                    _last_col->check_lasers[ii];
            in_state->state.laser_range_array[ii] =
                    _last_col->laser_range_array[ii];
            in_state->state.laser_status_array[ii] =
                    _last_col->laser_status_array[ii];
        }

        in_state->state.escape_count = _last_col->escape_count;
        in_state->state.escape_dist = _last_col->escape_dist;
        in_state->state.escape_angle = _last_col->escape_angle;

    #elif defined(STATEDATA_NAV)
        // TIME
        in_state->state.on_time = millis();
        // MOVE
        in_state->state.wheel_speed_left = _move_manager->get_encoder_speed_left();
        in_state->state.wheel_speed_right = _move_manager->get_encoder_speed_right();
        // IMU
        in_state->state.IMU_heading = _IMU->get_head_angle();
        in_state->state.IMU_pitch = _IMU->get_pitch_angle();
        in_state->state.IMU_roll = _IMU->get_roll_angle();
        // Navigation
        in_state->state.nav_pos_x = _navigator->get_pos_x();
        in_state->state.nav_pos_y = _navigator->get_pos_y();
        in_state->state.nav_vel_x = _navigator->get_vel_x();
        in_state->state.nav_vel_y = _navigator->get_vel_y();
        in_state->state.nav_vel_c = _navigator->get_vel_c();
        in_state->state.nav_head = _navigator->get_heading();
    #else
        // TIME
        in_state->state.on_time = millis();
        // MOOD
        in_state->state.mood = _mood_manager->get_mood();
        in_state->state.mood_score = _mood_manager->get_mood_score();
        // TASK
        in_state->state.task = _task_manager->get_task();
        // MOVE
        in_state->state.move_basic = _move_manager->get_basic_move();
        in_state->state.move_compound = _move_manager->get_compound_move();
        in_state->state.set_forward_speed = _move_manager->get_forward_speed();
        in_state->state.wheel_speed_left =
            _move_manager->get_encoder_left()->get_smooth_speed_mmps();
        in_state->state.wheel_speed_right =
            _move_manager->get_encoder_right()->get_smooth_speed_mmps();
        in_state->state.wheel_encoder_count_left =
            _move_manager->get_encoder_left()->get_count();
        in_state->state.wheel_encoder_count_right =
            _move_manager->get_encoder_right()->get_count();

    #endif
  }

  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  CollisionManager* _collision_manager = NULL;
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  IMUSensor* _IMU = NULL;
  Navigation* _navigator = NULL;
  SLastCollision* _last_col = NULL;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _enabled = true;
  bool _start_flag = true;

  // DATA PACKET
  UDataPacket _curr_state;
  bool _data_switch = false;

  // I2C VARIABLES
  uint16_t _I2C_time = STATEDATA_UPD_TIME;
  Timer _I2C_timer = Timer();
  Timer _send_timer = Timer();
};
#endif
