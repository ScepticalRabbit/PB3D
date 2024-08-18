//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "TaskPounce.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskPounce::TaskPounce(CollisionManager* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
            Speaker* inSpeaker){
    _collisionObj = inCollision;
    _mood_manager = inMood;
    _task_manager = inTask;
    _move_manager = inMove;
    _speakerObj = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskPounce::begin(){
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskPounce::update(){
    if(!_enabled){return;}

    if(_task_manager->getNewTaskFlag()){
        _startAllFlag = true;
    }
}

//---------------------------------------------------------------------------
// Pounce! - called during the main during decision tree
void TaskPounce::seekAndPounce(){
    if(!_enabled){return;}

    if(_startAllFlag){
        Serial.println("START ALL.");
        _startAllFlag = false;
        _startAll();
    }

    if(_state != _prevState){
        _prevState = _state;
        Serial.print("STATE = ");
        if(_state == POUNCE_SEEK){Serial.print("SEEK");}
        else if(_state == POUNCE_LOCKON){Serial.print("LOCK");}
        else if(_state == POUNCE_RUN){Serial.print("RUN");}
        else if(_state == POUNCE_REALIGN){Serial.print("REALIGN");}
        else{Serial.print("UNKNOWN");}
        Serial.println();
    }

    //-------------------------------------------------------------------------
    // POUNCE DECISION TREE
    // 1) Look for target
    // 2) Lock on to target
    // 3) Run to target
    // 4) Spin random number of degrees 90-270 and repeat
    if(_state == POUNCE_SEEK){
        _seekTarget();
    }
    else if(_state == POUNCE_LOCKON){
        _lockOn();
    }
    else if(_state == POUNCE_RUN){
        _runToTarget();
    }
    else if(_state == POUNCE_REALIGN){
        _realign();
    }
    else{
        _move_manager->stop();
    }
}

//---------------------------------------------------------------------------
// Get, set and reset
void TaskPounce::reset(){
    _startAll();
    _startAllFlag = true;
}

void TaskPounce::collisionResetToRealign(){
    if(_state == POUNCE_RUN){
        _realignStartFlag = true;
        _state = POUNCE_REALIGN;
    }
}

void TaskPounce::setRealignCent(int16_t inAng){
    _realignAngCent = inAng;
    _realignAngMin = _realignAngCent-_realignAngDev;
    _realignAngMax = _realignAngCent+_realignAngDev;
}

//---------------------------------------------------------------------------
// PRIVATE FUNCTIONS

//---------------------------------------------------------------------------
// START
void TaskPounce::_startAll(){
_state = POUNCE_SEEK;
// Seek
_seekStart = true;
_move_manager->reset_look();
// Lock On
_lockStartFlag = true;
// Run to Target
_runStartFlag = true;
// Realign
_realignStartFlag = true;
setRealignCent(180);
}

//---------------------------------------------------------------------------
// SEEK
void TaskPounce::_seekTarget(){
if(_seekStart){
    _seekStart = false;
    _move_manager->reset_look();
    _move_manager->reset_PIDs();
    _collisionObj->set_enabled_flag(false);
}
_collisionObj->set_enabled_flag(false); // Disable collision detection

// TASK LEDS
uint8_t seekCol = 1;
if(_move_manager->get_look_curr_ang_ind() == 0){
    //_task_manager->taskLEDCSV(_seekCol,_seekCol,255,_lowSat,255,255);
    _task_manager->taskLEDCSV(_seekCol,_seekCol,255,_lowSat,0,255);
}
else if(_move_manager->get_look_curr_ang_ind() == 1){
    _task_manager->taskLEDCSV(_seekCol,_seekCol,_lowSat,_lowSat,255,255);
}
else if(_move_manager->get_look_curr_ang_ind() == 2){
    //_task_manager->taskLEDCSV(_seekCol,_seekCol,_lowSat,255,255,255);
    _task_manager->taskLEDCSV(_seekCol,_seekCol,_lowSat,255,255,0);
}
else if(_move_manager->get_look_curr_ang_ind() == 3){
    _task_manager->taskLEDCSV(_seekCol,_seekCol,_lowSat,_lowSat,255,255);
}
else{
    _task_manager->taskLEDOff();
}

// Move to different angles and take measurements.
if(!_move_manager->get_look_finished()){
    _move_manager->look_around();

    if(_measCurInd < _measNumVals){
    // Get to the set point and start the measurement timer
    if(!_measFlag){
        if(_move_manager->get_pos_PID_attained_set_point()){
        //Serial.print("SP ATTAINED for "),Serial.println(_measCurInd);
        _measFlag = true;
        _measTimer.start(_measPrePauseTime);
        }
        else if(_move_manager->look_is_paused()){
        //Serial.print("SP TIMEOUT for "),Serial.println(_measCurInd);
        _measFlag = true;
        _measTimer.start(_measPrePauseTime);
        }
    }

    // If measurement timer is finished take a measurement
    if(_measFlag && _measTimer.finished()){
        _measTimer.start(_measInterval);
        int16_t sample = _collisionObj->get_ultrasonic_range_mm();
        _measSumForAvg = _measSumForAvg + sample;

        Serial.print("Sample "), Serial.print(_measCountForAvg);
        Serial.print(" = "), Serial.print(sample), Serial.print(" mm");
        Serial.println();

        _measCountForAvg++;

        if(_measCountForAvg >= _measNumForAvg){
        _measVec[_measCurInd] = _measSumForAvg/_measNumForAvg;

        Serial.print("Measurement Avg. "),Serial.print(_measCurInd);
        Serial.print(" = "), Serial.print(_measVec[_measCurInd]), Serial.print(" mm");
        Serial.println();

        _measFlag = false;
        _measCurInd++;

        _measSumForAvg = 0;
        _measCountForAvg = 0;

        // Measurement complete - force move to next pos
        _move_manager->force_look_move();
        }
    }
    }
}
else{ // EXIT CONDITION: LOOK FINISHED, TO NEXT STATE
    _measCurInd = 0;
    _state = POUNCE_LOCKON; // Move to next state
    _move_manager->reset_look();
}
}

//---------------------------------------------------------------------------
// LOCK
void TaskPounce::_lockOn(){
    if(_lockStartFlag){
        _lockStartFlag = false;

        // Decide on target
        _lockValidRangeCount = 0;
        for(int8_t ii=0; ii < _measNumVals; ii++){
        if((_measVec[ii] >= _lockLimRangeMin)&&(_measVec[ii] <= _lockLimRangeMax)){
            _lockValidRangeCount++;

            if(_measVec[ii] < _lockValidRangeMin){
            _lockValidRangeMin = _measVec[ii];
            _lockValidRangeMinInd = ii;
            }
            if(_measVec[ii] > _lockValidRangeMax){
            _lockValidRangeMax = _measVec[ii];
            _lockValidRangeMaxInd = ii;
            }
        }
        }
        // LOCK ON ANGLE/RANGE
        _lockOnAng = _measAngs[_lockValidRangeMinInd];
        _lockOnRange = float(_measVec[_lockValidRangeMinInd]);
        _lockOnTimer.start(_lockSpoolUpTime);  // Start timer

        _move_manager->reset_PIDs(); // Reset PIDs
        _collisionObj->set_enabled_flag(false); // Disable collisition detection

        // DEBUG: Lock on start
        Serial.println("LOCK ON: Start");
    }
    _collisionObj->set_enabled_flag(false); // Disable collision detection

    // Decide on a target
    // 1) If all ranges less than min range - REALIGN
    // 2) If all ranges greater than max range - GO TO CLOSEST
    // 3) Go through valid ranges and go to the nearest one - GO TO CLOSEST
    if(_lockValidRangeCount == 0){
        _state = POUNCE_REALIGN;
    }
    else{
        // Spool up - flash lights and wag tail - TODO
        _task_manager->taskLEDCSV(_lockCol,_lockCol,_lowSat,_lowSat,255,255);

        // Turn to target
        _move_manager->turn_to_angle_ctrl_pos(_lockOnAng);

        // EXIT CONDITION: SET POINT REACHED
        if(_move_manager->get_pos_PID_attained_set_point()){
        _state = POUNCE_RUN;
        }

        // EXIT CONDITION: TIMEOUT
        if(_lockOnTimer.finished()){
        _state = POUNCE_RUN;
        }
    }
}

//---------------------------------------------------------------------------
// RUN
void TaskPounce::_runToTarget(){
    if(_runStartFlag){
        _runStartFlag = false;

        // Re-calc timeout based on the measured range
        _runTimeout = int16_t(((_lockOnRange-float(_runRangeLim))/_runSpeed)*1000.0)+500;
        _runTimer.start(_runTimeout); // Start timer

        // Calculate encoder counts to get to target
        int32_t runEncCounts = int32_t((_lockOnRange-float(_runRangeLim))/(_move_manager->get_encoder_mm_per_count()));
        int32_t encAvgCounts = (_move_manager->get_encoder_count_left()+_move_manager->get_encoder_count_right())/2;
        _runEndEncCount = encAvgCounts+runEncCounts;

        _collisionObj->set_enabled_flag(true); // Re-enable collision detection
        // DEBUG: Run to start
        Serial.println("RUN: Start");
        Serial.print("RUN: Timeout = ");
        Serial.println(_runTimeout);
        Serial.print("RUN: Enc. Counts = ");
        Serial.println(runEncCounts);
    }

    // TASK LEDS
    _task_manager->taskLEDCSV(_runCol,_runCol,_lowSat,_lowSat,255,255);

    // EXIT CONDITION: Found target
    if(_collisionObj->get_ultrasonic_range_mm() <= _runRangeLim){
        _state = POUNCE_REALIGN;
        Serial.println("RUN END: US Range");
    }
    else if((_move_manager->get_encoder_count_left() >= _runEndEncCount) || (_move_manager->get_encoder_count_right() >= _runEndEncCount)){
        _state = POUNCE_REALIGN;
        Serial.println("RUN END: Enc Counts");
    }
    else{ // Go forward fast
        _move_manager->forward(_runSpeed);
    }

    // EXIT CONDITION: TIMEOUT
    if(_runTimer.finished()){
        _state = POUNCE_REALIGN;
        Serial.println("RUN END: Timer");
    }
}

//---------------------------------------------------------------------------
// REALIGN
void TaskPounce::_realign(){
    if(_realignStartFlag){
        _realignStartFlag = false;

        // Set to the initial state
        _realignState = 0;

        // Generate angle and start timers
        _realignAng = float(random(_realignAngMin,_realignAngMax));
        _realignTimer.start(_realignPrePauseTime);

        _collisionObj->set_enabled_flag(false); // Disable collision detection
        _move_manager->reset_PIDs();  // Reset move PIDs

        // DEBUG: Run to start
        Serial.println("REALIGN: Start, Pre-pause");
    }
    _collisionObj->set_enabled_flag(false); // Disable collision detection

    // TASK LEDS
    _task_manager->taskLEDCSV(_realignCol,_realignCol,_lowSat,_lowSat,255,255);

    // Move to the randomly generate angle to reorient
    if(_realignState == 0){ // Pre-pause
        _move_manager->stop();

        if(_realignTimer.finished()){
        _realignTimer.start(_realignTimeout);
        _realignState++;
        Serial.print("REALIGN: Moving to ");
        Serial.print(_realignAng);
        Serial.println(" deg");
        }
    }
    else if(_realignState == 1){// Move to angle
        _move_manager->turn_to_angle_ctrl_pos(_realignAng);

        // If angle is obtained or timeout go back to the start
        if(_move_manager->get_pos_PID_attained_set_point() || _realignTimer.finished()){
        _realignTimer.start(_realignPostPauseTime);
        _realignState++;
        Serial.println("REALIGN: Post pause");
        }
    }
    else if(_realignState == 2){// Post-pause
        _move_manager->stop();

        if(_realignTimer.finished()){
        _realignTimer.start(_realignTimeout);
        _realignState++;

        _state = POUNCE_SEEK;
        reset(); // Go back to start
        Serial.println("REALIGN: Finished, Reset");
        }
    }
    else{
        _move_manager->stop();
    }
    /*
    if(_realignPrePauseTimer.finished()){
        _move_manager->turnToAnglePosCtrl(_realignAng);

        // If angle is obtained or timeout go back to the start
        if(_move_manager->get_pos_PID_attained_set_point() || _realignTimer.finished()){
        _realignPostPauseTimer.start(_realignPostPauseTime);
        }
    }
    else if(_realignPrePauseTimer.finished() && _realignPostPauseTimer.finished()){
        _state = POUNCE_SEEK;
        reset(); // Go back to start
        Serial.println("REALIGN: Reset");
    }
    else{
        // Stop moving and wait for motors/momentum to settle
        _move_manager->stop();
    }
    */
}