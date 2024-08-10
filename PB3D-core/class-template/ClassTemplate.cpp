//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "ClassTemplate.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
ClassTemp::ClassTemp(Collision* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
            Speaker* inSpeaker){
_collisionObj = inCollision;
_moodObj = inMood;
_taskObj = inTask;
_moveObj = inMove;
_speakerObj = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called during SETUP
void ClassTemp::begin(){

}

//---------------------------------------------------------------------------
// UPDATE: called during LOOP
void ClassTemp::update(){
    if(!_isEnabled){return;}

    if(_taskObj->getNewTaskFlag()){
        _startFlag = true;
    }

}

//---------------------------------------------------------------------------
// DOSOMETHING - called during the main during decision tree
void ClassTemp::doSomething(){
    if(!_isEnabled){return;}

    if(_startFlag){
        _startFlag = false;
    }

}


