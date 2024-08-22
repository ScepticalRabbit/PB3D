#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskInteract.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskInteract.h"


TaskInteract::TaskInteract(MoodManager* mood, TaskManager* task,
                           MoveManager* move, Speaker* speaker,
                           TaskDance* dance, PatSensor* pat_sens){
    _mood_manager = mood;
    _task_manager = task;
    _move_manager = move;
    _task_dance = dance;
    _speaker = speaker;
    _pat_sensor = pat_sens;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskInteract::begin(){
    _ask_squeak_timer.start(0);
    _ask_wiggle_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskInteract::update(){
    if(!_enabled){return;}

    // SENSOR: Check for start of pat
    if(_pat_sensor->get_button_two_flag()){
        _task_manager->set_task(TASK_INTERACT);
        _task_manager->set_task_duration(_pat_timeout);
    }
}

//---------------------------------------------------------------------------
// INTERACT - called during the main during decision tree
void TaskInteract::interact(){
    // Set the LEDs on every loop
    _task_manager->task_LED_interact();

    // If this is the first time we enter the function set key variables
    if(_interact_start){
        _interact_start = false;

        _move_manager-> stop();
        _move_manager->reset_submove_timer();

        _pat_sensor->reset();
        _pat_sensor->set_buttons_enabled(false);
        _pat_timeout_timer.start(_pat_timeout);

        _ask_start = true;
        _ask_squeak_timer.start(_ask_squeak_interval);
        _ask_wiggle_timer.start(_askWiggleDuration);

        _speaker->reset();
    }

    // Set the speaker codes on every loop
    uint8_t in_codes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    _speaker->set_sound_codes(in_codes,4);
    //uint16_t in_freqs[]  = {NOTE_C5,NOTE_C7,NOTE_C4,NOTE_G7,0,0,0,0};
    uint16_t in_freqs[]  = {NOTE_B4,NOTE_B6,NOTE_B3,NOTE_F7,0,0,0,0};
    uint16_t in_durs[]   = {200,200,300,200,0,0,0,0};
    _speaker->set_sound_freqs(in_freqs,8);
    _speaker->set_sound_durations(in_durs,8);

    // Check if we need to ask for a pat again
    if(_ask_squeak_timer.finished()){
        _ask_start = true;
        _ask_squeak_timer.start(_ask_squeak_interval);
        _ask_wiggle_timer.start(_askWiggleDuration);

        // Reset the speaker
        _speaker->reset();
    }

    if(_ask_start){
        // If we are done 'asking' reset the flag
        if(_ask_wiggle_timer.finished()){
        _ask_start = false;
        }
        else{
        // Otherwise 'wiggle' to ask for a pat
        _move_manager->wiggle(_ask_wiggle_left_dur,_ask_wiggle_right_dur);
        }
    }
    else{
        // Wait for a pat until next wiggle time
        _move_manager->stop();
    }

    //-----------------------------------------------------------------------
    // ACCEPT PATS
    _pat_sensor->accept_pats();

    if(_pat_sensor->get_pat_finished()){
        // INTERACT EXIT CONDITION - reset start flag
        _interact_start = true;

        // Reset the pat sensor
        _pat_sensor->reset();
        _pat_sensor->set_buttons_enabled(true);

        // Update mood to happy
        int8_t prob = random(0,100);
        if(prob <= 80){
        _mood_manager->set_mood(MOOD_HAPPY);
        }
        else{
        _mood_manager->set_mood(MOOD_NEUTRAL);
        }
        _mood_manager->inc_mood_score();

        // Update task to dance
        _task_manager->set_task(TASK_DANCE);
        // Overide default task duration to be a specific number of bars
        _task_manager->set_task_duration(round(4*_task_dance->get_dance_bar_ms()));
        _task_manager->set_dance_update_flag(false);
        _task_dance->set_start_flag(true);
        _task_dance->set_speaker_flag(true);
    }

    // Check for timeout, if so set mood to sad and explore
    if(_pat_timeout_timer.finished()){
        // INTERACT EXIT CONDITION - reset start flag
        _interact_start = true;

        // Reset the pat sensor
        _pat_sensor->reset();
        _pat_sensor->set_buttons_enabled(true);

        // Update mood to sad
        int8_t prob = random(0,100);
        if(prob <= 75){
        _mood_manager->set_mood(MOOD_SAD);
        }
        else{
        _mood_manager->set_mood(MOOD_NEUTRAL);
        }
        _mood_manager->dec_mood_score();

        // Update task to explore
        _task_manager->set_task(TASK_EXPLORE);
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void TaskInteract::set_start_interact_flag(bool start){
    _interact_start = start;
    _pat_sensor->set_pat_flag(start);
}
