//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskRest.h"


TaskRest::TaskRest(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, Speaker* inSpeaker){
    _mood_manager = inMood;
    _task_manager = inTask;
    _move_manager = inMove;
    _speaker = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskRest::begin(){
    _timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskRest::update(){
    if(_task_manager->get_new_task_flag()){
        reset();
        _speaker->reset();
    }
}

//---------------------------------------------------------------------------
// REST
void TaskRest::rest(){
    uint8_t in_codes[] = {SPEAKER_SNORE,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
    _speaker->set_sound_codes(in_codes,4);

    _move_manager->stop();

    if(_timer.finished()){
        if(_rest_LED_increase){

            _rest_LED_val = _rest_LED_val+1;

            // If we are above the max value reset and decrease
            if(_rest_LED_val>=_rest_LED_max){
                _rest_LED_val = _rest_LED_max;
                _rest_LED_increase = false;
                _speaker->reset();
            }
            }
            else{
            _rest_LED_val = _rest_LED_val-1;

            if(_rest_LED_val<=_rest_LED_min){
                _rest_LED_val = _rest_LED_min;
                _rest_LED_increase = true;
            }
        }

        _timer.start(_rest_update_time);
        _task_manager->task_LED_rest(_rest_LED_val);
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void TaskRest::reset(){
    _timer.start(0);
    _rest_LED_val = _rest_LED_max;
    _rest_LED_increase = false;
    _speaker->reset();
}
