//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskDance.h"

TaskDance::TaskDance(MoodManager* mood, TaskManager* task,
                     MoveManager* move, Speaker* speaker){
    _mood_manager = mood;
    _task_manager = task;
    _move_manager = move;
    _speaker = speaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskDance::begin(){
    _dance_start = true;
    _timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every loop
void TaskDance::update(){
    // Get new task flag reset after first main loop update
    if(_task_manager->get_new_task_flag()){
        // Reset the dance start flag in case this is the current task.
        _dance_start = true;

        if(_task_manager->get_task() == TASK_DANCE){
        _generate_tempo();
        _generate_dance();
        _task_manager->set_dance_duration(_dance_duration);
        _task_manager->set_task_duration(_dance_duration);
        }
    }
}

//---------------------------------------------------------------------------
// DANCE
void TaskDance::dance(){
    // At the start of the dance set and reset key variables
    if(_dance_start){
        _start_dance();
    }

    // Set the task LEDs on every loop
    _task_manager->task_LED_dance();

    // If needed set the speaker flags on every loop
    if(_speaker_flag){
        uint8_t in_codes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
        _speaker->set_sound_codes(in_codes,4);
    }

    // Update the dance move at given time interval, should be 1/4 note
    if(_timer.finished()){
        _update_dance();
    }

    // MAIN DANCE DECISION TREE
    if(_dance_curr_move == DANCE_STOP){
        _move_manager->stop();
    }
    else if(_dance_curr_move == DANCE_FORBACK){
        //Serial.println("DANCE_FORBACK");
        _move_manager->forward_back(int(_dance_4note_ms),int(_dance_4note_ms));
    }
    else if(_dance_curr_move == DANCE_CIRCLE){
        //Serial.println("DANCE_CIRCLE");
        _move_manager->circle();
    }
    else if(_dance_curr_move == DANCE_TURN){
        //Serial.println("DANCE_TURN");
        if(_dance_turn_dir == MOVE_B_RIGHT){
        //Serial.println("DANCE TURN RIGHT.");
        _move_manager->turn_to_angle_ctrl_speed(90.0);
        }
        else{
        //Serial.println("DANCE TURN LEFT.");
        _move_manager->turn_to_angle_ctrl_speed(-90.0);
        }
    }
    else if(_dance_curr_move == DANCE_SPIN){
        //Serial.println("DANCE_SPIN");
        if(_dance_spin_dir == MOVE_B_RIGHT){
        _move_manager->right();
        }
        else{
        _move_manager->left();
        }
    }
    else{
        //Serial.println("DANCE_WIGGLE");
        _move_manager->wiggle(int(_dance_4note_ms),int(_dance_4note_ms));
    }
}

//---------------------------------------------------------------------------
// PRIVATE FUNCTIONS
void TaskDance::_start_dance(){
    _dance_start = false;
    // Set initial variables
    _timer.start(round(_dance_bar_ms));
    _dance_curr_bar = 0;
    _dance_move_ind = 0;
    _dance_curr_move = _dance_move_vec[_dance_move_ind];
    // RESET MOVE?

    // Increase mood when dance starts
    _mood_manager->inc_mood_score();

    // Set speaker
    if(_speaker_flag){
        _speaker->reset();
        uint8_t in_codes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
        _speaker->set_sound_codes(in_codes,4);
        uint16_t in_freqs[]  = {NOTE_G4,NOTE_G7,NOTE_G5,NOTE_G7,0,0,0,0};
        uint16_t in_durs[]   = {200,150,300,150,0,0,0,0};
        _speaker->set_sound_freqs(in_freqs,8);
        _speaker->set_sound_durations(in_durs,8);
    }
}

void TaskDance::_generate_tempo(){
    _dance_bpm = float(random(_dance_bpm_min,_dance_bpm_max)); // NOTE: random num between (min,max-1)
    _dance_4note_ms = (60.0/_dance_bpm)*1000.0;
    _dance_bar_ms = _dance_4note_ms*4.0;
    _dance_duration = uint32_t(_dance_bar_ms*float(_dance_num_bars));
}

void TaskDance::_generate_dance(){
    // Random move that recurs throughout the dance
    uint8_t recur_move = random(1,3);  // NOTE: random num between (min,max-1)
    // Bars on which the recurrent move occurs, 0=B1 and B3, 1=B2 and B4
    uint8_t recur_beat = random(0,2); // NOTE: random num between (min,max-1)
    // 1 = Full 8 bar dance, 2 = Repeat 4 bar dance
    uint8_t recur_bars = random(1,3); // NOTE: random num between (min,max-1)

    recur_move = 1;
    recur_beat = 0;
    //recur_bars = 2;

    for(uint8_t ii=0 ; ii < _dance_num_moves ; ii++){
        if((recur_bars == 2)&&(ii >= (_dance_num_moves/recur_bars))){
        _dance_move_vec[ii] = _dance_move_vec[ii-(_dance_num_moves/recur_bars)];
        }
        else{
        // If this is the recurring move set it, else generate
        if((ii%2)==recur_beat){
            _dance_move_vec[ii] = recur_move;
        }
        else{
            _dance_move_vec[ii] = random(1,DANCE_NUM_MOVES);
        }
        }
    }
    }

void TaskDance::_update_dance(){
    _timer.start(round(_dance_bar_ms));

    // Update the dance move
    _dance_move_ind = _dance_move_ind+1;
    if(_dance_move_ind >= _dance_num_moves){
        _dance_move_ind = 0;
    }
    _dance_curr_move = _dance_move_vec[_dance_move_ind];

    // Update the dance bar
    _dance_curr_bar = _dance_curr_bar+1;
    // If the dance is over reset the count
    if(_dance_curr_bar >= _dance_num_bars){
        _dance_start = true;
        _dance_curr_bar = 0;
    }

    // Update turn/spin direction
    if(_dance_move_ind == 0){
        _dance_turn_dir = MOVE_B_LEFT;
        _dance_spin_dir = MOVE_B_LEFT;
    }
    else if(_dance_move_vec[_dance_move_ind-1] == DANCE_TURN){
        if(_dance_turn_dir == MOVE_B_RIGHT){
        _dance_turn_dir = MOVE_B_LEFT;
        }
        else{
        _dance_turn_dir = MOVE_B_RIGHT;
        }
    }
    else if(_dance_move_vec[_dance_move_ind-1] == DANCE_SPIN){
        if(_dance_spin_dir = MOVE_B_RIGHT){
        _dance_spin_dir = MOVE_B_LEFT;
        }
        else{
        _dance_spin_dir = MOVE_B_RIGHT;
        }
    }

    // Reset the speaker to sync with dance bars
    _speaker->reset();
}

