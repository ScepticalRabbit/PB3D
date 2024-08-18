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
    _collisionObj = inCollision;
    _task_manager = inTask;
    _move_manager = inMove;
    _speakerObj = inSpeaker;
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

    if(_task_manager->getNewTaskFlag()){
        _pauseDur = random(_pauseDurMin,_pauseDurMax);
        _pauseTimer.start(_pauseDur);
    }
}

//---------------------------------------------------------------------------
// PAUSE
void TaskPause::pause(){
    if(!_enabled){return;}

    if(_pauseTimer.finished()){
        _task_manager->forceUpdate();
    }
    else{
        _task_manager->taskLEDPause(_pauseDur);
        _collisionObj->set_enabled_flag(false);
        _move_manager->stop();
    }
}