//==============================================================================
// PB3D: A pet robot that is 3D printed
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
CollisionManager::CollisionManager(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove){
    _mood_manager = inMood;
    _task_manager = inTask;
    _move_manager = inMove;

    _escaper.set_move_obj(inMove);

    for(uint8_t ii=0 ; ii < LASER_COUNT ; ii++){
        _check_lasers[ii] = 0;
    }
    for(uint8_t ii=0 ; ii < BUMP_COUNT ; ii++){
        _check_bumpers[ii] = 0;
    }
}

//------------------------------------------------------------------------------
// BEGIN: called once during SETUP
void CollisionManager::begin(){
    _bumper_timer.start(0);
    _slow_down_timer.start(0);

    _laser_manager.begin();
    _bumpers.begin();

    // Flag to turn off ultrasonic ranger to help with timing interruptions
    //_ultrasonic_ranger.set_enabled_flag(false);
}

//-----------------------------------------------------------------------------
// UPDATE: called during every LOOP
//-----------------------------------------------------------------------------
void CollisionManager::update(){
    //uint32_t startTime = micros();

    // If a new task is generated turn back on collision detecttion
    if(_task_manager->getNewTaskFlag()){
        set_enabled_flag(true);
    }

    // COLLISION SENSOR: Run laser ranging
    _laser_manager.update();

    // COLLISION SENSOR: Bumper Switches slaved to Nervous System
    if(_bumper_timer.finished()){
        _bumper_timer.start(_bumper_update_time);

        _bumpers.update();

        if(_bumpers.get_bump_thres_check()){
            _mood_manager->decMoodScore();
            _bumpers.reset_bump_count();
        }
    }

    // COLLISION DETECTION & DECISION
    // NOTE: this sets the collision flags that control escape behaviour!
    if(_check_timer.finished()){
        _check_timer.start(_check_interval);
        _update_check_vec(); // Sets collision detection flag

        if(_collision_slow_down && _slow_down_timer.finished()){
            _slow_down_timer.start(_slow_down_int);
            _move_manager->set_speed_by_col_code(true);
        }
        else if(!_collision_slow_down && _slow_down_timer.finished()){
            _move_manager->set_speed_by_col_code(false);
        }
    }
    if(!_slow_down_timer.finished()){
        _move_manager->set_speed_by_col_code(true);
    }

    // DISABLED: If collision detection is turned off set flags to false and return
    // Doing this last allows ranges to update but resets flags
    if(!_enabled){reset_flags();}

    //uint32_t endTime = micros();
    //Serial.println(endTime-startTime);
}

//---------------------------------------------------------------------------
// Get, set and reset
//---------------------------------------------------------------------------
bool CollisionManager::get_altitude_flag(){
    if(_laser_manager.get_collision_code(LASER_ALT) > DANGER_NONE){
        return true;
    }
    else{
        return false;
    }
}

void CollisionManager::reset_flags(){
    _collision_detected = false;
    _collision_slow_down = false;
    _bumpers.reset();
}

//-----------------------------------------------------------------------------
void CollisionManager::set_escape_start(){
    _update_check_vec();    // Check all collision sensors - used for decision tree
    _update_escape_decision();
}

//-----------------------------------------------------------------------------
void CollisionManager::escape(){
    _escaper.escape();
}

//-----------------------------------------------------------------------------
bool CollisionManager::get_escape_flag(){
    return _escaper.get_escape_flag();
}

//-----------------------------------------------------------------------------
int8_t CollisionManager::get_escape_turn(){
    _update_check_vec();    // Check all collision sensors - used for decision tree
    return _escaper.get_escape_turn(_check_lasers);
}

//---------------------------------------------------------------------------
void CollisionManager::_update_check_vec(){

    _collision_detected = false;
    _collision_slow_down = false;

    for(uint8_t ii=0 ; ii<BUMP_COUNT ; ii++){
        _check_bumpers[ii] = _bumpers.get_collision_code(EBumpCode(ii));

        if(_check_bumpers[ii] >= DANGER_CLOSE){
            _collision_slow_down = true;
        }
    }


    for(uint8_t ii=0 ; ii<LASER_COUNT ; ii++){
        _check_lasers[ii] = _laser_manager.get_collision_code(ELaserIndex(ii));
        if(_check_lasers[ii] >= DANGER_SLOW){
            _collision_slow_down = true;
        }
        if(_check_lasers[ii] >= DANGER_FAR){
            _collision_detected = true;
        }
    }
}

//-----------------------------------------------------------------------------
// ESCAPE DECISION TREE
void CollisionManager::_update_escape_decision(){
    // Forward to escaper
    _escaper.update_escape_decision(_check_lasers);

    for(uint8_t ii=0 ; ii<BUMP_COUNT ; ii++){
        _last_col.check_bumpers[ii] = _bumpers.get_collision_code(EBumpCode(ii));
    }

    for(uint8_t ii=0 ; ii<LASER_COUNT ; ii++){
        _last_col.check_lasers[ii] = _check_lasers[ii];
        _last_col.laser_range_array[ii] = _laser_manager.get_range(
                                                        ELaserIndex(ii));
        _last_col.laser_status_array[ii] = _laser_manager.get_status(
                                                        ELaserIndex(ii));
    }


    _last_col.escape_count = _escaper.get_escape_count();

    // Plot debug information
    #if defined(COLL_DEBUG_DECISIONTREE)
        Serial.println();
        Serial.println(F("======================================="));

        Serial.println(F("CheckVec=[BL,BR,US,LL,LR,LU,LD,]"));
        Serial.print("CheckVec=[");
        for(uint8_t ii=0;ii<_checkNum;ii++){
            Serial.print(" ");Serial.print(_check_lasers[ii]);Serial.print(",");
        }
        Serial.println("]");
        Serial.println();

        Serial.print("US= "); Serial.print(_last_col.ultrasonic_range); Serial.println(" mm");

        Serial.print("LL= "); Serial.print(_last_col.LSRStatusL); Serial.print(", ");
        Serial.print(_last_col.LSRRangeL); Serial.println(" mm");
        Serial.print("LR= " ); Serial.print(_last_col.LSRStatusR); Serial.print(", ");
        Serial.print(_last_col.LSRRangeR); Serial.println(" mm");
        Serial.print("LU= "); Serial.print(_last_col.LSRStatusU); Serial.print(", ");
        Serial.print(_last_col.LSRRangeU); Serial.println(" mm");
        Serial.print("LD= "); Serial.print(_last_col.LSRStatusD); Serial.print(", ");
        Serial.print(_last_col.LSRRangeD); Serial.println(" mm");

        Serial.println();
        Serial.print("Esc,Count="); Serial.print(_last_col.escape_count);
        Serial.print(", Dist="); Serial.print(_last_col.escape_dist);
        Serial.print(", Angle="); Serial.print(_last_col.escape_angle);
        Serial.println();

        Serial.println(F("======================================="));
        Serial.println();
    #endif
}



