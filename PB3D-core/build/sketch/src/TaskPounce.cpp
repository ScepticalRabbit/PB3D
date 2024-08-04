#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskPounce.cpp"
//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TaskPounce
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "TaskPounce.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR - pass in pointers to main objects and other sensors
TaskPounce::TaskPounce(CollisionManager* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, 
            Speaker* inSpeaker){
    _collisionObj = inCollision;
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void TaskPounce::begin(){
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
void TaskPounce::update(){
    if(!_isEnabled){return;}

    if(_taskObj->getNewTaskFlag()){
        _startAllFlag = true;
    }
}

//---------------------------------------------------------------------------
// Pounce! - called during the main during decision tree
void TaskPounce::seekAndPounce(){   
    if(!_isEnabled){return;}

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
        _moveObj->stop();
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
_moveObj->resetLook();
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
    _moveObj->resetLook();
    _moveObj->resetPIDs();
    _collisionObj->setEnabledFlag(false);
}
_collisionObj->setEnabledFlag(false); // Disable collision detection 

// TASK LEDS
uint8_t seekCol = 1;
if(_moveObj->getLookCurrAngInd() == 0){
    //_taskObj->taskLEDCSV(_seekCol,_seekCol,255,_lowSat,255,255);
    _taskObj->taskLEDCSV(_seekCol,_seekCol,255,_lowSat,0,255);
}
else if(_moveObj->getLookCurrAngInd() == 1){
    _taskObj->taskLEDCSV(_seekCol,_seekCol,_lowSat,_lowSat,255,255);
}
else if(_moveObj->getLookCurrAngInd() == 2){
    //_taskObj->taskLEDCSV(_seekCol,_seekCol,_lowSat,255,255,255);
    _taskObj->taskLEDCSV(_seekCol,_seekCol,_lowSat,255,255,0);
}
else if(_moveObj->getLookCurrAngInd() == 3){
    _taskObj->taskLEDCSV(_seekCol,_seekCol,_lowSat,_lowSat,255,255);
}
else{
    _taskObj->taskLEDOff();
}
    
// Move to different angles and take measurements.
if(!_moveObj->getLookFinished()){
    _moveObj->lookAround();

    if(_measCurInd < _measNumVals){
    // Get to the set point and start the measurement timer
    if(!_measFlag){
        if(_moveObj->getPosPIDAttainSP_Both()){
        //Serial.print("SP ATTAINED for "),Serial.println(_measCurInd);
        _measFlag = true;
        _measTimer.start(_measPrePauseTime);
        }
        else if(_moveObj->lookIsPaused()){
        //Serial.print("SP TIMEOUT for "),Serial.println(_measCurInd);
        _measFlag = true;
        _measTimer.start(_measPrePauseTime);
        }
    }
        
    // If measurement timer is finished take a measurement
    if(_measFlag && _measTimer.finished()){
        _measTimer.start(_measInterval);
        int16_t sample = _collisionObj->getUSRangeMM();
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
        _moveObj->forceLookMove();
        }
    }
    }
}
else{ // EXIT CONDITION: LOOK FINISHED, TO NEXT STATE
    _measCurInd = 0;
    _state = POUNCE_LOCKON; // Move to next state
    _moveObj->resetLook();
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
        
        _moveObj->resetPIDs(); // Reset PIDs
        _collisionObj->setEnabledFlag(false); // Disable collisition detection

        // DEBUG: Lock on start
        Serial.println("LOCK ON: Start");
    }
    _collisionObj->setEnabledFlag(false); // Disable collision detection 

    // Decide on a target
    // 1) If all ranges less than min range - REALIGN
    // 2) If all ranges greater than max range - GO TO CLOSEST
    // 3) Go through valid ranges and go to the nearest one - GO TO CLOSEST
    if(_lockValidRangeCount == 0){
        _state = POUNCE_REALIGN; 
    }
    else{
        // Spool up - flash lights and wag tail - TODO
        _taskObj->taskLEDCSV(_lockCol,_lockCol,_lowSat,_lowSat,255,255);

        // Turn to target
        _moveObj->turnToAngleCtrlPos(_lockOnAng);

        // EXIT CONDITION: SET POINT REACHED
        if(_moveObj->getPosPIDAttainSP_Both()){
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
        int32_t runEncCounts = int32_t((_lockOnRange-float(_runRangeLim))/(_moveObj->getEncMMPerCount()));
        int32_t encAvgCounts = (_moveObj->getEncCountL()+_moveObj->getEncCountR())/2;
        _runEndEncCount = encAvgCounts+runEncCounts;
        
        _collisionObj->setEnabledFlag(true); // Re-enable collision detection    
        // DEBUG: Run to start
        Serial.println("RUN: Start");
        Serial.print("RUN: Timeout = ");
        Serial.println(_runTimeout);
        Serial.print("RUN: Enc. Counts = ");
        Serial.println(runEncCounts);
    }

    // TASK LEDS
    _taskObj->taskLEDCSV(_runCol,_runCol,_lowSat,_lowSat,255,255);

    // EXIT CONDITION: Found target
    if(_collisionObj->getUSRangeMM() <= _runRangeLim){
        _state = POUNCE_REALIGN;
        Serial.println("RUN END: US Range");
    }
    else if((_moveObj->getEncCountL() >= _runEndEncCount) || (_moveObj->getEncCountR() >= _runEndEncCount)){
        _state = POUNCE_REALIGN;
        Serial.println("RUN END: Enc Counts");
    }
    else{ // Go forward fast
        _moveObj->forward(_runSpeed);
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
        
        _collisionObj->setEnabledFlag(false); // Disable collision detection 
        _moveObj->resetPIDs();  // Reset move PIDs

        // DEBUG: Run to start
        Serial.println("REALIGN: Start, Pre-pause");
    }
    _collisionObj->setEnabledFlag(false); // Disable collision detection 

    // TASK LEDS
    _taskObj->taskLEDCSV(_realignCol,_realignCol,_lowSat,_lowSat,255,255);

    // Move to the randomly generate angle to reorient
    if(_realignState == 0){ // Pre-pause
        _moveObj->stop();
        
        if(_realignTimer.finished()){
        _realignTimer.start(_realignTimeout);
        _realignState++;
        Serial.print("REALIGN: Moving to ");
        Serial.print(_realignAng);
        Serial.println(" deg");
        }
    }
    else if(_realignState == 1){// Move to angle
        _moveObj->turnToAngleCtrlPos(_realignAng);

        // If angle is obtained or timeout go back to the start
        if(_moveObj->getPosPIDAttainSP_Both() || _realignTimer.finished()){
        _realignTimer.start(_realignPostPauseTime);
        _realignState++;
        Serial.println("REALIGN: Post pause");
        }
    }
    else if(_realignState == 2){// Post-pause
        _moveObj->stop();
        
        if(_realignTimer.finished()){
        _realignTimer.start(_realignTimeout);
        _realignState++;

        _state = POUNCE_SEEK;
        reset(); // Go back to start
        Serial.println("REALIGN: Finished, Reset");
        }
    }
    else{
        _moveObj->stop();
    }
    /*
    if(_realignPrePauseTimer.finished()){
        _moveObj->turnToAnglePosCtrl(_realignAng);

        // If angle is obtained or timeout go back to the start
        if(_moveObj->getPosPIDAttainSP_Both() || _realignTimer.finished()){
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
        _moveObj->stop();
    }
    */
}