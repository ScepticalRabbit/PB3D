//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskPickedUp.h"

//------------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskPickedUp::TaskPickedUp(CollisionManager* collision, MoodManager* mood,
                          TaskManager* task, MoveManager* move,
                          Speaker* speaker, PatSensor* pat_sens,
                          MultiIOExpander* multi_expander){
    _collision_manager = collision;
    _mood_manager = mood;
    _task_manager = task;
    _move_manager = move;
    _speaker = speaker;
    _pat_sensor = pat_sens;
    _multi_expander = multi_expander;
}

//------------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskPickedUp::begin(){
    Wire.beginTransmission(ADDR_TOUCHSENS);
    if(Wire.endTransmission() != 0){
        Serial.println(F("TASKPICKEDUP: Failed to initialise touch sensor."));
    }
    else{
        Serial.println(F("TASKPICKEDUP: Touch sensor initialised."));
    }

    _rand_pause_call = random(_rand_pause_call_min,_rand_pause_call_max);
    _rand_pause_panic = random(_rand_pause_panic_min,_rand_pause_panic_max);
    _purr_on_time = random(_purr_on_time_min,_purr_on_time_max);
    _purr_off_time = random(_purr_off_time_min,_purr_off_time_max);

    _call_timer.start(0);
    _purr_timer.start(0);
}

//------------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskPickedUp::update(){
    if(!enabled){return;}

    // Check the altirude to see if PB has been picked up
    if(_collision_manager->get_altitude_flag() && !_picked_up){
        _picked_up = true;
        _start_picked_up = true;

        // If picked up turn off collision detection
        _collision_manager->enabled = false;

        // Based on the current task set the panic flag
        if((_task_manager->get_task() == TASK_INTERACT) ||
            (_task_manager->get_task() == TASK_DANCE)){
            _panic_flag = false;
        }
        else{
            _panic_flag = true;
        }

        _task_manager->set_task(TASK_PICKEDUP);
    }
}

//---------------------------------------------------------------------------
// PICKED UP - called during the main during decision tree
void TaskPickedUp::pickedUp(){
    // If the sensor wasn't found then do nothing
    if(!enabled){return;}

    //--------------------------------------------
    // START ONLY
    if(_start_picked_up){
        _start_picked_up = false;
        _pat_flag = false;
        _pat_complete = false;
        _exit_flag = false;
        _pat_sensor->reset();
        _pat_sensor->set_buttons_enabled(false);

        if(_panic_flag){
        _mood_manager->dec_mood_score();
        }
    }

    //--------------------------------------------
    // LOOP ACTIONS: Switch based on panic flag
    if(_panic_flag){
        // MOOD UPDATE (PANIC): 50/50 angry or scared
        int8_t prob = random(0,100);
        if(prob <= 50){_mood_manager->set_mood(MOOD_SCARED);}
        else{_mood_manager->set_mood(MOOD_ANGRY);}

        // TaskManager LEDS - Panic
        _task_manager->task_LED_pickedup_panic();

        // Wiggle to get free
        _move_manager->wiggle(_panic_wiggle_left_dur,_panic_wiggle_right_dur);

        // Speaker updates
        uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_BEEP,SPEAKER_SLIDE,SPEAKER_BEEP};
        _speaker->set_sound_codes(inCodes,4);
        _speaker->set_sound_freqs(_panic_freqs,8);
        _panic_durs[_pause_ind] = _rand_pause_panic;
        _speaker->set_sound_durations(_panic_durs,8);

        _call_update_time = _speaker->get_total_sound_duration();
    }
    else{
        // MoodManager - Neutral
        _mood_manager->set_mood(MOOD_NEUTRAL);
        // TaskManager LEDS - Ok
        _task_manager->task_LED_pickedup_Ok();

        // Stop motor
        _move_manager->stop();

        // Speaker updates
        uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_SLIDE};
        _speaker->set_sound_codes(inCodes,4);
        _speaker->set_sound_freqs(_call_freqs,8);
        _call_durs[_pause_ind] = _rand_pause_call;
        _speaker->set_sound_durations(_call_durs,8);

        _call_update_time = _speaker->get_total_sound_duration();
    }

    //--------------------------------------------
    // PAT SENSOR
    // Update the pat sensor
    _pat_sensor->accept_pats();

    // If patted once set the class flag
    if(_pat_sensor->get_pat_count() >= 1){
        _pat_flag = true;
    }

    if(_pat_sensor->get_pat_finished()){
        // Set internal switch for exit condition
        _pat_complete = true;
        // Reset the pat sensor
        _pat_sensor->reset();

        // MOOD UPDATE: 75% chance go to happy
        int8_t prob = random(0,100);
        if(prob<75){_mood_manager->set_mood(MOOD_HAPPY);}
        _mood_manager->inc_mood_score();
    }

    // If picked up turn off collision detection
    _collision_manager->enabled = false;

    //--------------------------------------------
    // VIBRATION MOTOR: PURRING
    if(_pat_flag){
        // If patted stop panicking
        _panic_flag = false;

        if(_purr_timer.finished()){
            _purr_on_time = random(_purr_on_time_min,_purr_on_time_max);
            _purr_off_time = random(_purr_off_time_min,_purr_off_time_max);

            _purr_on = !_purr_on;
            if(_purr_on){
                _purr_timer.start(_purr_on_time);
            }
            else{
                _purr_timer.start(_purr_off_time);
            }
        }

        if(_purr_on){
            _multi_expander->digital_write(GPIO_PURR_VIBE,HIGH);
        }
        else{
            _multi_expander->digital_write(GPIO_PURR_VIBE,LOW);
        }

    }

    //--------------------------------------------
    // SPEAKER UPDATE
    if(_call_timer.finished()){
        // Generate random time parameters for speaker
        _rand_pause_call = random(_rand_pause_call_min,_rand_pause_call_max);
        _rand_pause_panic = random(_rand_pause_panic_min,_rand_pause_panic_max);

        if(_panic_flag){
        _panic_durs[_pause_ind] = _rand_pause_panic;
        _speaker->set_sound_durations(_panic_durs,8);
        }
        else{
        _call_durs[_pause_ind] = _rand_pause_call;
        _speaker->set_sound_durations(_call_durs,8);
        }

        // Restart call timer with new times
        _call_update_time = _speaker->get_total_sound_duration();
        _call_timer.start(_call_update_time);

        // Reset the speaker
        _speaker->reset();
    }

    //--------------------------------------------
    // EXIT TIMER
    if(!_collision_manager->get_altitude_flag()){
        if(!_exit_timer_on){
        _exit_timer_on = true;
        _exit_timer.start(_exit_time);
        }
        else if(_exit_timer_on && _exit_timer.finished()){
        _exit_flag = true;
        }
    }
    else{
        _exit_timer_on = false;
        _exit_timer.start(_exit_time);
    }

    //--------------------------------------------
    // EXIT CONDITIONS
    if(_exit_flag && _picked_up){
        _picked_up = false;
        _pat_sensor->set_buttons_enabled(true);
        _collision_manager->enabled = true;
        _task_manager->set_task(TASK_PAUSE);

        // Based on the state on exit set different conditions
        int8_t prob = random(0,100);
        if(_panic_flag){
            if(prob<50){_mood_manager->set_mood(MOOD_SCARED);}
            else{_mood_manager->set_mood(MOOD_ANGRY);}
        }
        else if(_pat_complete){
            if(prob<80){_mood_manager->set_mood(MOOD_HAPPY);}
            else{_mood_manager->set_mood(MOOD_NEUTRAL);}
        }
        else{
            _mood_manager->set_mood(MOOD_NEUTRAL);
        }
    }
}