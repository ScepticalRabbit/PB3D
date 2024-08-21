#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskRest.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskRest.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
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
    // If the task has changed then modify other classes as needed
    if(_task_manager->get_new_task_flag()){
        reset();
        _speaker->reset();
    }
}

//---------------------------------------------------------------------------
// REST
void TaskRest::rest(){
    // Reset the speaker flags
    uint8_t inCodes[] = {SPEAKER_SNORE,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
    _speaker->set_sound_codes(inCodes,4);

    // Stop moving while sleeping
    _move_manager->stop();

    if(_timer.finished()){
        if(_restLEDIncrease){
        // If we are increasing increment the LED intensity
        _restLEDVal = _restLEDVal+1;

        // If we are above the max value reset and decrease
        if(_restLEDVal>=_restLEDMax){
            _restLEDVal = _restLEDMax;
            _restLEDIncrease = false;
            _speaker->reset();
        }
        }
        else{
        // If we are decreasing decrement the LED intensity
        _restLEDVal = _restLEDVal-1;

        // If we are below the min intensity reset and increase
        if(_restLEDVal<=_restLEDMin){
            _restLEDVal = _restLEDMin;
            _restLEDIncrease = true;
        }
        }
        // Restart the timerr
        _timer.start(_restUpdateTime);
        // Set the current LED value
        _task_manager->task_LED_rest(_restLEDVal);
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void TaskRest::reset(){
    _timer.start(0);
    _restLEDVal = _restLEDMax;
    _restLEDIncrease = false;
    _speaker->reset();
}
