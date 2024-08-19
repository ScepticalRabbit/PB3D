//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskPickedUp.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskPickedUp::TaskPickedUp(CollisionManager* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
            Speaker* inSpeaker, PatSensor* inPatSens){
    _collision_manager = inCollision;
    _mood_manager = inMood;
    _task_manager = inTask;
    _move_manager = inMove;
    _speaker = inSpeaker;
    _patSensObj = inPatSens;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskPickedUp::begin(){
    // Ping the touch sensor to see if it is connected
    Wire.beginTransmission(ADDR_TOUCHSENS);
    if(Wire.endTransmission() != 0){
        Serial.println(F("TASKPICKEDUP: Failed to initialise touch sensor."));
    }
    else{
        Serial.println(F("TASKPICKEDUP: Touch sensor initialised."));
    }

    // Generate all random parameters
    _randPauseCall = random(_randPauseCallMin,_randPauseCallMax);
    _randPausePanic = random(_randPausePanicMin,_randPausePanicMax);
    _purrOnTime = random(_purrOnTimeMin,_purrOnTimeMax);
    _purrOffTime = random(_purrOffTimeMin,_purrOffTimeMax);

    // Start all timers to trip on first use
    _callTimer.start(0);
    _purrTimer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskPickedUp::update(){
    // If the sensor wasn't found then do nothing
    if(!_enabled){return;}

    // Check the altirude to see if PB has been picked up
    if(_collision_manager->get_altitude_flag() && !_isPickedUp){
        _isPickedUp = true;
        _startPickedUpFlag = true;

        // If picked up turn off collision detection
        _collision_manager->set_enabled_flag(false);

        // Based on the current task set the panic flag
        if((_task_manager->get_task() == TASK_INTERACT)||(_task_manager->get_task() == TASK_DANCE)){
        _panicFlag = false;
        }
        else{
        _panicFlag = true;
        }

        // Set the task to "picked up"
        _task_manager->set_task(TASK_PICKEDUP);
    }
}

//---------------------------------------------------------------------------
// PICKED UP - called during the main during decision tree
void TaskPickedUp::pickedUp(){
    // If the sensor wasn't found then do nothing
    if(!_enabled){return;}

    //--------------------------------------------
    // START ONLY
    if(_startPickedUpFlag){
        _startPickedUpFlag = false;
        _pat_flag = false;
        _pat_complete = false;
        _exitFlag = false;
        _patSensObj->reset();
        _patSensObj->set_buttons_enabled(false);

        if(_panicFlag){
        _mood_manager->dec_mood_score();
        }
    }

    //--------------------------------------------
    // LOOP ACTIONS: Switch based on panic flag
    if(_panicFlag){
        // MOOD UPDATE (PANIC): 50/50 angry or scared
        int8_t prob = random(0,100);
        if(prob <= 50){_mood_manager->set_mood(MOOD_SCARED);}
        else{_mood_manager->set_mood(MOOD_ANGRY);}

        // TaskManager LEDS - Panic
        _task_manager->task_LED_pickedup_panic();

        // Wiggle to get free
        _move_manager->wiggle(_panicWiggleLeftDur,_panicWiggleRightDur);

        // Speaker updates
        uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_BEEP,SPEAKER_SLIDE,SPEAKER_BEEP};
        _speaker->setSoundCodes(inCodes,4);
        _speaker->setSoundFreqs(_panicFreqs,8);
        _panicDurs[_pauseInd] = _randPausePanic;
        _speaker->setSoundDurs(_panicDurs,8);

        _callUpdateTime = _speaker->getTotalSoundDur();
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
        _speaker->setSoundCodes(inCodes,4);
        _speaker->setSoundFreqs(_callFreqs,8);
        _callDurs[_pauseInd] = _randPauseCall;
        _speaker->setSoundDurs(_callDurs,8);

        _callUpdateTime = _speaker->getTotalSoundDur();
    }

    //--------------------------------------------
    // PAT SENSOR
    // Update the pat sensor
    _patSensObj->accept_pats();

    // If patted once set the class flag
    if(_patSensObj->get_pat_count() >= 1){
        _pat_flag = true;
    }

    if(_patSensObj->get_pat_finished()){
        // Set internal switch for exit condition
        _pat_complete = true;
        // Reset the pat sensor
        _patSensObj->reset();

        // MOOD UPDATE: 75% chance go to happy
        int8_t prob = random(0,100);
        if(prob<75){_mood_manager->set_mood(MOOD_HAPPY);}
        _mood_manager->inc_mood_score();
    }

    // If picked up turn off collision detection
    _collision_manager->set_enabled_flag(false);

    //--------------------------------------------
    // VIBRATION MOTOR: PURRING
    if(_pat_flag){
        // If patted stop panicking
        _panicFlag = false;

        if(_purrTimer.finished()){
        // Regenerate random purr times
        _purrOnTime = random(_purrOnTimeMin,_purrOnTimeMax);
        _purrOffTime = random(_purrOffTimeMin,_purrOffTimeMax);

        // Flip the purr switch
        _purrOnFlag = !_purrOnFlag;
        if(_purrOnFlag){
            _purrTimer.start(_purrOnTime);
            // Set byte to turn on purr sensor
            _sendByte = B00001111;
        }
        else{
            _purrTimer.start(_purrOffTime);
            // Set byte to turn on purr sensor
            _sendByte = B00001110;
        }
        }
        // Send byte to turn on/off pat sensor
        Wire.beginTransmission(ADDR_FOLLBOARD);
        Wire.write(_sendByte);
        Wire.endTransmission();
    }

    //--------------------------------------------
    // SPEAKER UPDATE
    if(_callTimer.finished()){
        // Generate random time parameters for speaker
        _randPauseCall = random(_randPauseCallMin,_randPauseCallMax);
        _randPausePanic = random(_randPausePanicMin,_randPausePanicMax);

        if(_panicFlag){
        _panicDurs[_pauseInd] = _randPausePanic;
        _speaker->setSoundDurs(_panicDurs,8);
        }
        else{
        _callDurs[_pauseInd] = _randPauseCall;
        _speaker->setSoundDurs(_callDurs,8);
        }

        // Restart call timer with new times
        _callUpdateTime = _speaker->getTotalSoundDur();
        _callTimer.start(_callUpdateTime);

        // Reset the speaker
        _speaker->reset();
    }

    //--------------------------------------------
    // EXIT TIMER
    if(!_collision_manager->get_altitude_flag()){
        if(!_exitTimerOn){
        _exitTimerOn = true;
        _exitTimer.start(_exitTime);
        }
        else if(_exitTimerOn && _exitTimer.finished()){
        _exitFlag = true;
        }
    }
    else{
        _exitTimerOn = false;
        _exitTimer.start(_exitTime);
    }

    //--------------------------------------------
    // EXIT CONDITIONS
    if(_exitFlag && _isPickedUp){
        _isPickedUp = false;
        // Re-enable pat sensor buttons
        _patSensObj->set_buttons_enabled(true);
        // Re-enable collision detection
        _collision_manager->set_enabled_flag(true);
        // Set task to pause
        _task_manager->set_task(TASK_PAUSE);

        // Based on the state on exit set different conditions
        int8_t prob = random(0,100);
        if(_panicFlag){
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