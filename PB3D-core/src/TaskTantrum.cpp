//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskTantrum.h"


TaskTantrum::TaskTantrum(MoodManager* mood, TaskManager* task,
                         MoveManager* move, Speaker* speaker){
    _mood_manager = mood;
    _move_manager = move;
    _task_manager = task;
    _move_manager = move;
    _speaker = speaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskTantrum::begin(){
    _tantrum_timer.start(0);
    _growl_timer.start(0);
}

//---------------------------------------------------------------------------
// TANTRUM
void TaskTantrum::chuck_tantrum(){
if(_start_tantrum){
    _start_tantrum = false;
    // Tantrum timer and completion
    _tantrum_complete = false;
    _tantrum_timer.start(_tantrum_duration);
    // Growl timer and flag
    _growl_on = true;
    _growl_timer.start(_tantrum_growl_duration);

    // MOOD UPDATE: 30% chance of angry
    int8_t prob = random(0,100);
    if(prob<30){_mood_manager->set_mood(MOOD_ANGRY);}
    _mood_manager->dec_mood_score();

    _speaker->reset();
    uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    _speaker->set_sound_codes(inCodes,4);
    uint16_t inDurs[]   = {600,300,600,300,0,0,0,0};
    _speaker->set_sound_durations(inDurs,8);
}

// Set the speaker codes on every loop
uint8_t inCodes[]   = {SPEAKER_GROWL,SPEAKER_GROWL,SPEAKER_OFF,SPEAKER_OFF};
_speaker->set_sound_codes(inCodes,4);

// Set the task LEDs on every loop regardless
_task_manager->task_LED_tantrum();

if(_growl_timer.finished()){
    _growl_on = !_growl_on;
    if(_growl_on){
    _growl_timer.start(_tantrum_growl_duration);
    _speaker->reset();
    }
    else{
    _growl_timer.start(_tantrum_growl_pause);
    }
}
if(_tantrum_timer.finished()){
    _tantrum_complete = true;
}
else{
    _move_manager->forward_back(_tantrum_FB_duration,_tantrum_FB_duration);
}
}

//---------------------------------------------------------------------------
// Get, set and reset
void TaskTantrum::set_start_tantrum(){
    _start_tantrum = true;
    _tantrum_complete = false;
}