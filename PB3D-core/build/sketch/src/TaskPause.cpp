#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskPause.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskPause.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskPause::TaskPause(CollisionManager* inCollision, TaskManager* inTask, MoveManager* inMove, Speaker* inSpeaker){
    _collision_manager = inCollision;
    _task_manager = inTask;
    _move_manager = inMove;
    _speaker = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskPause::begin(){
    _pauseTimer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskPause::update(){
    if(!_enabled){return;}

    if(_task_manager->get_new_task_flag()){
        _pauseDur = random(_pauseDurMin,_pauseDurMax);
        _pauseTimer.start(_pauseDur);
    }
}

//---------------------------------------------------------------------------
// PAUSE
void TaskPause::pause(){
    if(!_enabled){return;}

    if(_pauseTimer.finished()){
        _task_manager->force_update();
    }
    else{
        _task_manager->task_LED_pause(_pauseDur);
        _collision_manager->set_enabled_flag(false);
        _move_manager->stop();
    }
}