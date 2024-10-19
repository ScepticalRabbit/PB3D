//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include <Arduino.h>
#include "CollisionManager.h"

//------------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
CollisionManager::CollisionManager(MoodManager* mood,
                                   TaskManager* task,
                                   MoveManager* move,
                                   LaserManager* lasers,
                                   BumperSensor* bumpers){
    _mood_manager = mood;
    _task_manager = task;
    _move_manager = move;
    _laser_manager = lasers;
    _bumpers = bumpers;

    _escaper.set_move_obj(move);

    for(uint8_t ii=0 ; ii < LASER_COUNT ; ii++){
        _check_lasers[ii] = DANGER_NONE;
    }
    for(uint8_t ii=0 ; ii < BUMP_COUNT ; ii++){
        _check_bumpers[ii] = DANGER_NONE;
    }
}

//------------------------------------------------------------------------------
// BEGIN: called once during SETUP
void CollisionManager::begin(){
    _slow_down_timer.start(0);
}

//------------------------------------------------------------------------------
// UPDATE: called during every LOOP
void CollisionManager::update(){
    //uint32_t startTime = micros();

    // If a new task is generated turn back on collision detecttion
    if(_task_manager->get_new_task_flag()){
        enabled = true;
    }

    // COLLISION DETECTION & DECISION
    // NOTE: this sets the collision flags that control escape behaviour!
    if(_check_timer.finished()){
        _check_timer.start(_check_interval);

        _check_and_latch();

        if(_collision_slow_down_latch && _slow_down_timer.finished()){
            _slow_down_timer.start(_slow_down_int);
            _move_manager->set_speed_danger_multiplier(DANGER_CLOSE);
        }

        if(!_slow_down_timer.finished()){
            _move_manager->set_speed_danger_multiplier(DANGER_CLOSE);
        }
        else{
            _collision_slow_down_latch = false;
            _move_manager->set_speed_danger_multiplier(DANGER_NONE);
        }

        if(_bumpers->get_bump_thres_check()){
            _mood_manager->dec_mood_score();
            _bumpers->reset_bump_count();
        }
    }

    // DISABLED: If collision detection is turned off set flags to false and
    // return. Doing this last allows ranges to update but resets flags
    if(!enabled){reset_unlatch();}

    //uint32_t endTime = micros();
    //Serial.println(endTime-startTime);
}

//------------------------------------------------------------------------------
// Get, set and reset
bool CollisionManager::get_altitude_flag(){
    if(_laser_manager->get_collision_code(LASER_ALT) > DANGER_NONE){
        return true;
    }
    else{
        return false;
    }
}

void CollisionManager::reset_unlatch(){
    _collision_detected_latch = false;
    _collision_slow_down_latch = false;
    _bumpers->reset();
}

//-----------------------------------------------------------------------------
void CollisionManager::set_escape_start(){
    _check_and_latch();

    _escaper.update_escape_decision(_check_bumpers,_check_lasers);

    //--------------------------------------------------------------------------
    // Update last collision struct
    for(uint8_t ii=0 ; ii<BUMP_COUNT ; ii++){
        _last_col.check_bumpers[ii] = _bumpers->get_collision_code(EBumpCode(ii));
    }

    for(uint8_t ii=0 ; ii<LASER_COUNT ; ii++){
        _last_col.check_lasers[ii] = _check_lasers[ii];
        _last_col.laser_range_array[ii] = _laser_manager->get_range(
                                                        ELaserIndex(ii));
        _last_col.laser_status_array[ii] = _laser_manager->get_status(
                                                        ELaserIndex(ii));
    }

    _last_col.escape_count = _escaper.get_escape_count();
    //--------------------------------------------------------------------------

    #if defined(COLL_DEBUG_DECISIONTREE)
        Serial.println();
        Serial.println(F("======================================="));
        Serial.println(F("TODO"));
        Serial.println(F("======================================="));
        Serial.println();
    #endif
}

//------------------------------------------------------------------------------
void CollisionManager::escape(){
    _escaper.escape();
}

//------------------------------------------------------------------------------
bool CollisionManager::get_escape_state(){
    return _escaper.get_escape_state();
}

//------------------------------------------------------------------------------
EMoveBasic CollisionManager::get_escape_turn(){
    return _escaper.get_escape_turn();
}

//------------------------------------------------------------------------------
void CollisionManager::_check_and_latch(){

    for(uint8_t ii=0 ; ii<BUMP_COUNT ; ii++){
        _check_bumpers[ii] = _bumpers->get_collision_code(EBumpCode(ii));

        if(_check_bumpers[ii] >= DANGER_CLOSE){
            _collision_detected_latch = true;
        }
    }

    for(uint8_t ii=0 ; ii<LASER_COUNT ; ii++){
        _check_lasers[ii] = _laser_manager->get_collision_code(ELaserIndex(ii));
        if(_check_lasers[ii] >= DANGER_SLOW){
            _collision_slow_down_latch = true;
        }
        if(_check_lasers[ii] >= DANGER_FAR){
            _collision_detected_latch = true;
        }
    }
}



