//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskFindHuman.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskFindHuman::TaskFindHuman(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
            Speaker* inSpeaker, TaskInteract* inTInt){
    _mood_manager = inMood;
    _task_manager = inTask;
    _move_manager = inMove;
    _speakerObj = inSpeaker;
    _taskInteractObj = inTInt;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskFindHuman::begin(){
    delay(500);
    _presDetector = PresenceDetector(_movementSensor, _sensitivityPresence, _sensitivityMovement, _detectInterval);
    if (_movementSensor.initialize() == false){
        Serial.println(F("TASKFINDHUMAN: Failed to initialise presence sensor."));
        _is_enabled = false;
    }
    Serial.println(F("TASKFINDHUMAN: IR presence sensor initialised."));
    _IRPFlags1 = false; _IRPFlags2 = false; _IRPFlags3 = false; _IRPFlags4 = false;
    _sensUpdateTimer.start(0);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskFindHuman::update(){
    // If the human presence sensor wasn't found then do nothing
    if(!_is_enabled){
        return;
    }

    if(_task_manager->getNewTaskFlag()){
        _start_flag = true;
    }

    // SENSOR: IR Human Presence Sensor
    _presDetector.loop();
    if(_sensUpdateTimer.finished()){
        _sensUpdateHumanIRSensor();
        _sensUpdateTimer.start(_humIRSensUpdateTime);
    }
}

//---------------------------------------------------------------------------
// FIND HUMAN - called during task decision tree
void TaskFindHuman::findHuman(){
    // Set the LEDs on every loop regardless
    _task_manager->taskLEDFindHuman();

    // If the human presence sensor wasn't found then bypass the rest of the function
    if(!_is_enabled){
        _task_manager->forceUpdate();
        return;
    }

    // First time this is called as a new task reset some variables
    if(_start_flag){
        _start_flag = false;

        _speakerObj->reset();
        _callTimer.start(_callInterval);
    }


    // Set the speaker codes on every loop
    uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    _speakerObj->setSoundCodes(inCodes,4);
    uint16_t inFreqs[]  = {NOTE_A4,NOTE_A4,NOTE_A4,NOTE_CS7,0,0,0,0};
    uint16_t inDurs[]   = {300,0,200,0,0,0,0,0};
    _speakerObj->setSoundFreqs(inFreqs,8);
    _speakerObj->setSoundDurs(inDurs,8);

    //--------------------------------------------------------------------
    // Found you!
    //--------------------------------------------------------------------
    // NOTE SENSOR ORIENTATION DIFFERENT ON PB3D
    // Sensor 1 is pointing LEFT
    // Sensor 1: LEFT, Sensor 2: FRONT, Sensor 3: RIGHT, Sensor 4: BACK
    if(_IRPFlags1 && _IRPFlags2 && _IRPFlags3 && _IRPFlags4){
        // If all flags are tripped a human has been found!
        _task_manager->setTask(TASK_INTERACT);
        // Overide the default task duration
        _task_manager->setTaskDuration(_taskInteractObj->getTimeOut());
        _task_manager->taskLEDInteract();
        _taskInteractObj->setStartInteractFlag(true);
        // Update mood score
        _mood_manager->incMoodScore();
    }
    else{
        //--------------------------------------------------------------------
        // Speaker call = where you?
        //--------------------------------------------------------------------
        if(_callTimer.finished()){
            _speakerObj->reset();
            _callTimer.start(_callInterval);
        }

        //--------------------------------------------------------------------
        // Track based on IR sensor readings
        //--------------------------------------------------------------------
        // If grove connector points backwards then the sensors are:
        // 1: back, 2: left, 3: front, 4: right
        if(_diffIR13 > _sensitivityTrack){
            // Sensor 1 greater than sensor 3
            _move_manager->left();
        }
        else if(_diffIR13 < -_sensitivityTrack){
            // Sensor 3 greater than 1
            _move_manager->forward();
        }
        else if(_diffIR24 > _sensitivityTrack){
            // Sensor 2 greater than sensor 4
            _move_manager->left();
        }
        else if(_diffIR24 < -_sensitivityTrack){
            // Sensor 4 greater than sensor 2
            _move_manager->right();
        }
        else{
            _move_manager->update_move();
            _move_manager->go();
        }
    }
}

//---------------------------------------------------------------------------
void TaskFindHuman::_sensUpdateHumanIRSensor(){
    _presDetector.presentFullField(false);

    _IRtDer1 = _presDetector.getDerivativeOfIR1();
    _IRtDer2 = _presDetector.getDerivativeOfIR2();
    _IRtDer3 = _presDetector.getDerivativeOfIR3();
    _IRtDer4 = _presDetector.getDerivativeOfIR4();

    _diffIR13 = _presDetector.getDerivativeOfDiff13();
    _diffIR24 = _presDetector.getDerivativeOfDiff24();

    _IRPFlags1 = _presDetector.presentField1();
    _IRPFlags2 = _presDetector.presentField2();
    _IRPFlags3 = _presDetector.presentField3();
    _IRPFlags4 = _presDetector.presentField4();
}