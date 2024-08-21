//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------
#ifndef COLLISIONMANAGER_H
#define COLLISIONMANAGER_H

#include <Arduino.h>

#include "PB3DConstants.h"

#include "MoodManager.h"
#include "MoveManager.h"
#include "TaskManager.h"
#include "Timer.h"
#include "CollisionEscaper.h"
#include "LaserManager.h"
#include "BumperSensor.h"


// DEBUG Flags
//#define COLL_DEBUG_DECISIONTREE

//------------------------------------------------------------------------------
// LAST COLLISION: data structure
struct SLastCollision{
  uint8_t check_bumpers[BUMP_COUNT];
  uint8_t check_lasers[LASER_COUNT];
  int16_t laser_range_array[LASER_COUNT];
  uint8_t laser_status_array[LASER_COUNT];
  uint8_t escape_count = 0;
  float escape_dist = 0.0;
  float escape_angle = 0.0;
};

//------------------------------------------------------------------------------
// COLLISION MANAGER: class to handle object avoidance behaviour
class CollisionManager{
public:
  //----------------------------------------------------------------------------
  // CONSTRUCTOR: pass in pointers to main objects and other sensors
  CollisionManager(MoodManager* mood, TaskManager* task, MoveManager* move);

  //----------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //----------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //----------------------------------------------------------------------------
  // Get, set and reset
  bool get_enabled_flag(){return _enabled;}
  void set_enabled_flag(bool flag){_enabled = flag;}

  bool get_detected(){return _collision_detected;}

  uint16_t get_count(){return _collision_count;}
  void inc_count(){_collision_count++;}
  void reset_Count(){_collision_count = 0;}

  bool get_beepbeep_flag(){return _collision_beepbeep_flag;}
  void set_beepbeep_flag(bool flag){_collision_beepbeep_flag = flag;}

  bool get_bumper_flag(){return _bumpers.get_bump_flag();}

  int16_t get_laser_range(ELaserIndex _ind){
    return _laser_manager.get_range(_ind);
  }
  SLastCollision* get_last_collision(){return &_last_col;}

  uint8_t get_col_check(uint8_t pos){return _check_lasers[pos];}

  bool get_altitude_flag();
  void reset_flags();

  //----------------------------------------------------------------------------
  // Command forwarding to escaper
  void set_escape_start();
  void escape();
  bool get_escape_flag();
  int8_t get_escape_turn();

private:
  //----------------------------------------------------------------------------
  // Check all ranges and escape decision tree
  void _update_check_vec();
  void _update_escape_decision(); // Escape decision tree

  //----------------------------------------------------------------------------
  // CLASS VARIABLES
  // Main object and sensor pointers
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;

  // Collision management variables
  bool _enabled = true;
  bool _collision_detected = false; // Key flag controlling collision escape
  bool _collision_slow_down = false;

  uint16_t _half_body_leng_mm = 80;
  bool _collision_beepbeep_flag = false;
  uint16_t _collision_count = 0;

  CollisionEscaper _escaper = CollisionEscaper();
  LaserManager _laser_manager = LaserManager();
  BumperSensor _bumpers = BumperSensor();

  // Check flags for all collision sensors
  uint8_t _check_lasers[LASER_COUNT];
  uint8_t _check_bumpers[BUMP_COUNT];
  uint16_t _check_interval = 50;
  Timer _check_timer = Timer();

  // Time to slow down if collision sensor tripped
  uint16_t _slow_down_int = 500;
  Timer _slow_down_timer = Timer();

  int16_t _bumper_update_time = 101;
  Timer _bumper_timer = Timer();

  SLastCollision _last_col;
};
#endif // COLLISIONMANAGER_H
