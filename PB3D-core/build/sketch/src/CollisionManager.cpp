#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/CollisionManager.cpp"
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
}

//------------------------------------------------------------------------------
// BEGIN: called once during SETUP
void CollisionManager::begin(){
    _ultrasonicTimer.start(0);
    _bumperTimer.start(0);
    _slowDownTimer.start(0);

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

    // COLLISION SENSOR: Run ultrasonic ranging, based on update time interval
    if(_ultrasonicTimer.finished()){
        _ultrasonicTimer.start(_ultrasonicUpdateTime);
        _ultrasonic_ranger.update();
    }

    // COLLISION SENSOR: Run laser ranging
    _laser_manager.update();

    // COLLISION SENSOR: Bumper Switches slaved to Nervous System
    if(_bumperTimer.finished()){
        _bumperTimer.start(_bumperUpdateTime);

        _bumpers.update();

        if(_bumpers.get_bump_thres_check()){
            _mood_manager->decMoodScore();
            _bumpers.reset_bump_count();
        }
    }

    // COLLISION DETECTION & DECISION
    // NOTE: this sets the collision flags that control escape behaviour!
    if(_checkAllTimer.finished()){
        _checkAllTimer.start(_checkAllInt);
        _update_check_vec(); // Sets collision detection flag

        if(_collisionSlowDown && _slowDownTimer.finished()){
            _slowDownTimer.start(_slowDownInt);
            _move_manager->setSpeedByColFlag(true);
        }
        else if(!_collisionSlowDown && _slowDownTimer.finished()){
            _move_manager->setSpeedByColFlag(false);
        }
    }
    if(!_slowDownTimer.finished()){
        _move_manager->setSpeedByColFlag(true);
    }

    // DISABLED: If collision detection is turned off set flags to false and return
    // Doing this last allows ranges to update but resets flags
    if(!_is_enabled){reset_flags();}

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
    _collisionSlowDown = false;
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
bool CollisionManager::getEscapeFlag(){
    return _escaper.get_escape_flag();
}

//-----------------------------------------------------------------------------
int8_t CollisionManager::get_escape_turn(){
    _update_check_vec();    // Check all collision sensors - used for decision tree
    return _escaper.get_escape_turn(_checkVec);
}

//---------------------------------------------------------------------------
void CollisionManager::_update_check_vec(){
    // uint8_t _checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
    // NOTE: if's have to have most severe case first!

    // Bumper - Left
    _checkVec[0] = _bumpers.get_collision_code(0);
    // Bumper - Right
    _checkVec[1] = _bumpers.get_collision_code(1);

    // Ultrasonic Ranger
    _checkVec[2] = _ultrasonic_ranger.get_collision_code();

    // Laser - Left
    _checkVec[3] = _laser_manager.getColCodeL();
    // Laser - Right
    _checkVec[4] = _laser_manager.getColCodeR();
    // Laser - Up Angle
    _checkVec[5] = _laser_manager.getColCodeU();
    // Laser - Down Angle - TODO, fix this so we know which is which
    _checkVec[6] = _laser_manager.getColCodeD();

    // If anything is tripped set flags to true
    _collision_detected = false;
    _collisionSlowDown = false;
    for(uint8_t ii=0;ii<_checkNum;ii++){
        if(_checkVec[ii] >= DANGER_SLOWD){
            _collisionSlowDown = true;
        }
        if(_checkVec[ii] >= DANGER_FAR){
            _collision_detected = true;
        }
    }
}

//-----------------------------------------------------------------------------
// ESCAPE DECISION TREE
void CollisionManager::_update_escape_decision(){
    // Forward to escaper
    _escaper.update_escape_decision(_checkVec);

    // Update the last collision variable after the decision tree
    for(uint8_t ii=0;ii<_checkNum;ii++){
        _last_col.check_vec[ii] = _checkVec[ii];
    }
    _last_col.ultrasonic_range = _ultrasonic_ranger.get_range_mm();

    _last_col.LSRRangeL = _laser_manager.getRangeL();
    _last_col.LSRRangeR = _laser_manager.getRangeR();
    _last_col.LSRRangeU = _laser_manager.getRangeU();
    _last_col.LSRRangeD = _laser_manager.getRangeD();

    _last_col.LSRStatusL = _laser_manager.getStatusL();
    _last_col.LSRStatusR = _laser_manager.getStatusR();
    _last_col.LSRStatusU = _laser_manager.getStatusU();
    _last_col.LSRStatusD = _laser_manager.getStatusD();

    _last_col.escape_count = _escaper.get_escape_count();

    // Plot debug information
    #if defined(COLL_DEBUG_DECISIONTREE)
        Serial.println();
        Serial.println(F("======================================="));

        Serial.println(F("CheckVec=[BL,BR,US,LL,LR,LU,LD,]"));
        Serial.print("CheckVec=[");
        for(uint8_t ii=0;ii<_checkNum;ii++){
            Serial.print(" ");Serial.print(_checkVec[ii]);Serial.print(",");
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



