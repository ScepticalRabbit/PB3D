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
_collision_manager = inCollision;
_mood_manager = inMood;
_task_manager = inTask;
_move_manager = inMove;
_speaker = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called during SETUP
void ClassTemp::begin(){

}

//---------------------------------------------------------------------------
// UPDATE: called during LOOP
void ClassTemp::update(){
    if(!_enabled){return;}

    if(_task_manager->get_new_task_flag()){
        _start_flag = true;
    }

}

//---------------------------------------------------------------------------
// DOSOMETHING - called during the main during decision tree
void ClassTemp::doSomething(){
    if(!_enabled){return;}

    if(_start_flag){
        _start_flag = false;
    }

}


