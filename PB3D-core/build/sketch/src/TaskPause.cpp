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


TaskPause::TaskPause(CollisionManager* collision, TaskManager* task,
                     MoveManager* move, Speaker* speaker){
    _collision_manager = collision;
    _task_manager = task;
    _move_manager = move;
    _speaker = speaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskPause::begin(){
    _pause_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskPause::update(){
    if(!_enabled){return;}

    if(_task_manager->get_new_task_flag()){
        _pause_dur = random(_pause_dur_min,_pause_dur_max);
        _pause_timer.start(_pause_dur);
    }
}

//---------------------------------------------------------------------------
// PAUSE
void TaskPause::pause(){
    if(!_enabled){return;}

    if(_pause_timer.finished()){
        _task_manager->force_update();
    }
    else{
        _task_manager->task_LED_pause(_pause_dur);
        _collision_manager->set_enabled_flag(false);
        _move_manager->stop();
    }
}