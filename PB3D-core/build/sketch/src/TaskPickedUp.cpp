#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskPickedUp.cpp"
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
    _collisionObj = inCollision;
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
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
    if(!_is_enabled){return;}

    // Check the altirude to see if PB has been picked up
    if(_collisionObj->getAltFlag() && !_isPickedUp){
        _isPickedUp = true;
        _startPickedUpFlag = true;

        // If picked up turn off collision detection
        _collisionObj->set_enabled_flag(false);

        // Based on the current task set the panic flag
        if((_taskObj->getTask() == TASK_INTERACT)||(_taskObj->getTask() == TASK_DANCE)){
        _panicFlag = false;
        }
        else{
        _panicFlag = true;
        }

        // Set the task to "picked up"
        _taskObj->setTask(TASK_PICKEDUP);
    }
}

//---------------------------------------------------------------------------
// PICKED UP - called during the main during decision tree
void TaskPickedUp::pickedUp(){
    // If the sensor wasn't found then do nothing
    if(!_is_enabled){return;}

    //--------------------------------------------
    // START ONLY
    if(_startPickedUpFlag){
        _startPickedUpFlag = false;
        _patFlag = false;
        _patComplete = false;
        _exitFlag = false;
        _patSensObj->reset();
        _patSensObj->setButtonsEnabled(false);

        if(_panicFlag){
        _moodObj->decMoodScore();
        }
    }

    //--------------------------------------------
    // LOOP ACTIONS: Switch based on panic flag
    if(_panicFlag){
        // MOOD UPDATE (PANIC): 50/50 angry or scared
        int8_t prob = random(0,100);
        if(prob <= 50){_moodObj->setMood(MOOD_SCARED);}
        else{_moodObj->setMood(MOOD_ANGRY);}

        // TaskManager LEDS - Panic
        _taskObj->taskLEDPickedUpPanic();

        // Wiggle to get free
        _moveObj->wiggle(_panicWiggleLeftDur,_panicWiggleRightDur);

        // Speaker updates
        uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_BEEP,SPEAKER_SLIDE,SPEAKER_BEEP};
        _speakerObj->setSoundCodes(inCodes,4);
        _speakerObj->setSoundFreqs(_panicFreqs,8);
        _panicDurs[_pauseInd] = _randPausePanic;
        _speakerObj->setSoundDurs(_panicDurs,8);

        _callUpdateTime = _speakerObj->getTotalSoundDur();
    }
    else{
        // MoodManager - Neutral
        _moodObj->setMood(MOOD_NEUTRAL);
        // TaskManager LEDS - Ok
        _taskObj->taskLEDPickedUpOk();

        // Stop motor
        _moveObj->stop();

        // Speaker updates
        uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_SLIDE};
        _speakerObj->setSoundCodes(inCodes,4);
        _speakerObj->setSoundFreqs(_callFreqs,8);
        _callDurs[_pauseInd] = _randPauseCall;
        _speakerObj->setSoundDurs(_callDurs,8);

        _callUpdateTime = _speakerObj->getTotalSoundDur();
    }

    //--------------------------------------------
    // PAT SENSOR
    // Update the pat sensor
    _patSensObj->acceptPats();

    // If patted once set the class flag
    if(_patSensObj->getPatCount() >= 1){
        _patFlag = true;
    }

    if(_patSensObj->getPatFinished()){
        // Set internal switch for exit condition
        _patComplete = true;
        // Reset the pat sensor
        _patSensObj->reset();

        // MOOD UPDATE: 75% chance go to happy
        int8_t prob = random(0,100);
        if(prob<75){_moodObj->setMood(MOOD_HAPPY);}
        _moodObj->incMoodScore();
    }

    // If picked up turn off collision detection
    _collisionObj->set_enabled_flag(false);

    //--------------------------------------------
    // VIBRATION MOTOR: PURRING
    if(_patFlag){
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
        _speakerObj->setSoundDurs(_panicDurs,8);
        }
        else{
        _callDurs[_pauseInd] = _randPauseCall;
        _speakerObj->setSoundDurs(_callDurs,8);
        }

        // Restart call timer with new times
        _callUpdateTime = _speakerObj->getTotalSoundDur();
        _callTimer.start(_callUpdateTime);

        // Reset the speaker
        _speakerObj->reset();
    }

    //--------------------------------------------
    // EXIT TIMER
    if(!_collisionObj->getAltFlag()){
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
        _patSensObj->setButtonsEnabled(true);
        // Re-enable collision detection
        _collisionObj->set_enabled_flag(true);
        // Set task to pause
        _taskObj->setTask(TASK_PAUSE);

        // Based on the state on exit set different conditions
        int8_t prob = random(0,100);
        if(_panicFlag){
        if(prob<50){_moodObj->setMood(MOOD_SCARED);}
        else{_moodObj->setMood(MOOD_ANGRY);}
        }
        else if(_patComplete){
        if(prob<80){_moodObj->setMood(MOOD_HAPPY);}
        else{_moodObj->setMood(MOOD_NEUTRAL);}
        }
        else{
        _moodObj->setMood(MOOD_NEUTRAL);
        }
    }
}