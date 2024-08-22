#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskFindSound.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskFindSound.h"


TaskFindSound::TaskFindSound(MoodManager* mood, TaskManager* task,
                            MoveManager* move, Speaker* speaker){
    _mood_manager = mood;
    _task_manager = task;
    _move_manager = move;
    _speaker = speaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskFindSound::begin(){
    // Send the byte flag back
    Wire.beginTransmission(ADDR_FOLLOW_XIAO_2);
    Wire.write(_send_byte);
    Wire.endTransmission();

    // Start all timers
    _sens_update_timer.start(0);
    _clap_enable_timer.start(0);
    _env_samp_timer.start(0);
    _call_timer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during each loop to update variables
void TaskFindSound::update(){
    // If the sensor wasn't found then do nothing
    if(!_enabled){return;}

    // SENSOR: Ask follower Xiao for sound data
    if(_sens_update_timer.finished()){
        _sens_update_timer.start(_sens_update_time);

        // Request ear state data from follower board
        _I2C_read_ear_state();

        // Send the byte flag back
        _I2C_send_byte();

        if((_ear_state==EAR_COM_FORWARD)||(_ear_state==EAR_COM_LEFT)||(_ear_state==EAR_COM_RIGHT)){
            _clap_count++;
        }

        // DEBUG: Print to Serial
        #ifdef DEBUG_TASKFINDSOUND
        Serial.print("E-STATE: ");
        Serial.print(_ear_state), Serial.print(", M: ");
        if(_ear_state==EAR_COM_FORWARD){Serial.print("F");}
        else if(_ear_state==EAR_COM_LEFT){Serial.print("L");}
        else if(_ear_state==EAR_COM_RIGHT){Serial.print("R");}
        else if(_ear_state==EAR_COM_SENV){Serial.print("E");}
        else{Serial.print("N");}
        Serial.println();
        #endif
    }

    if(_task_manager->get_task() != TASK_FINDSOUND){
        // ENABLE: If X claps in this interval then start finding sound
        if(_clap_enable_timer.finished()){
        _clap_enable_timer.start(_clap_enable_update_time);

        if(_clap_count >= _clap_thres){
            _task_manager->set_task(TASK_FINDSOUND);
        }
        _clap_count = 0;
        }

        // SAMPLE ENV: Resample environment to prevent false trips of the sensor
        if(_env_samp_timer.finished()){
            _env_samp_timer.start(_env_samp_update_time);
            _I2C_send_samp_env_flag();
        }
    }
    else{
        _clap_count = 0;
    }

    // NEW TASK: if task is new set the start flag
    if(_task_manager->get_new_task_flag()){
        _start_flag = true;
    }
}

//---------------------------------------------------------------------------
// FINDSOUND - called during task decision tree
void TaskFindSound::find_sound(){
    // Set the LEDs on every loop regardless
    _task_manager->task_LED_find_sound();

    // If the sensor wasn't found then sxit the function
    if(!_enabled){return;}

    //--------------------------------------------------------------------
    // START
    // First time this is called as a new task reset some variables
    if(_start_flag){
        _start_flag = false;

        // Send the flag to sample environment
        _I2C_send_samp_env_flag();

        _speaker->reset();
        _call_timer.start(_call_interval);
    }

    //--------------------------------------------------------------------
    // SPEAKER: call = where are you?
    // Set the speaker codes on every loop
    // uint8_t in_codes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    uint8_t in_codes[]   = {SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
    _speaker->set_sound_codes(in_codes,4);
    uint16_t in_freqs[]  = {NOTE_A4,NOTE_G4,NOTE_G4,NOTE_A6,0,0,0,0};
    uint16_t in_durs[]   = {300,0,200,0,0,0,0,0};
    _speaker->set_sound_freqs(in_freqs,8);
    _speaker->set_sound_durations(in_durs,8);


    if(_call_timer.finished()){
        _speaker->reset();
        _call_timer.start(_call_interval);
    }

    //--------------------------------------------------------------------
    // FIND SOUND: Track based on ear state from Xiao
    // EAR STATE = 0: uncertain, 1: forward, 2: left, 3: right
    if(_ear_state == EAR_COM_LEFT){
        //_move_manager->left();
        //_move_manager->forward_left(_speed_diff_leftright);
        _move_manager->forward_left_diff_frac(_speed_diff_frac_leftright);
    }
    else if(_ear_state == EAR_COM_RIGHT){
        //_move_manager->right();
        //_move_manager->forward_right(_speed_diff_leftright);
        _move_manager->forward_right_diff_frac(_speed_diff_frac_leftright);
    }
    else if(_ear_state == EAR_COM_FORWARD){
        _move_manager->forward();
    }
    else if(_ear_state == EAR_COM_SENV){
        _move_manager->forward();
    }
    else{
        //_move_manager->update_move();
        //_move_manager->go();
        _move_manager->forward();
    }
}

//---------------------------------------------------------------------------
// PRIVATE FUNCTIONS
void TaskFindSound::_I2C_send_byte(){
    Wire.beginTransmission(ADDR_FOLLOW_XIAO_2);
    Wire.write(_send_byte);
    Wire.endTransmission();
}

void TaskFindSound::_I2C_send_samp_env_flag(){
    byte sampEnvByte = _send_byte | B01000000;
    Wire.beginTransmission(ADDR_FOLLOW_XIAO_2);
    Wire.write(sampEnvByte);
    Wire.endTransmission();
}

void TaskFindSound::_I2C_read_ear_state(){
    Wire.requestFrom(ADDR_FOLLOW_XIAO_2, 1);
    while (Wire.available()){
        _ear_state = Wire.read();
    }
}
