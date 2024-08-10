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
    _collisionObj = inCollision;
    _taskObj = inTask;
    _moveObj = inMove;
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
    if(!_is_enabled){return;}

    if(_taskObj->getNewTaskFlag()){
        _pauseDur = random(_pauseDurMin,_pauseDurMax);
        _pauseTimer.start(_pauseDur);
    }
}

//---------------------------------------------------------------------------
// PAUSE
void TaskPause::pause(){
    if(!_is_enabled){return;}

    if(_pauseTimer.finished()){
        _taskObj->forceUpdate();
    }
    else{
        _taskObj->taskLEDPause(_pauseDur);
        _collisionObj->set_enabled_flag(false);
        _moveObj->stop();
    }
}