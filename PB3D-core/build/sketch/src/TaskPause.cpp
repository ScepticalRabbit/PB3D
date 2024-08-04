#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskPause.cpp"
//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TaskPause
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
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
    if(!_isEnabled){return;}

    if(_taskObj->getNewTaskFlag()){
        _pauseDur = random(_pauseDurMin,_pauseDurMax);
        _pauseTimer.start(_pauseDur);
    }
}

//---------------------------------------------------------------------------
// PAUSE
void TaskPause::pause(){
    if(!_isEnabled){return;}

    if(_pauseTimer.finished()){
        _taskObj->forceUpdate();
    }
    else{
        _taskObj->taskLEDPause(_pauseDur);
        _collisionObj->setEnabledFlag(false);
        _moveObj->stop();
    }
}