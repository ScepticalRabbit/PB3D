#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/MoveManager.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "MoveManager.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//---------------------------------------------------------------------------
MoveManager::MoveManager(Adafruit_MotorShield* AFMS, Encoder* encL, Encoder* encR){
    _AFMS = AFMS;
    _encoder_L = encL;
    _encoder_R = encR;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
//---------------------------------------------------------------------------
void MoveManager::begin(){
    // Start the motor shield object
    _AFMS->begin();  // create with the default frequency 1.6KHz
    // M1 is the right motor, M2 is the left motor
    _motorR = _AFMS->getMotor(1);
    _motorL = _AFMS->getMotor(2);
    // Set the speed to start, from 0 (off) to 255 (max  speed)
    _motorR->setSpeed(_defForwardPWR);
    _motorR->run(FORWARD);
    _motorR->run(RELEASE);
    _motorL->setSpeed(_defForwardPWR);
    _motorL->run(FORWARD);
    _motorL->run(RELEASE);

    // Randomly generate a move type and start the timer
    _moveCompound = random(0,_moveCompoundCount);
    _moveUpdateTime = random(_moveUpdateMinTime,_moveUpdateMaxTime);
    _moveTimer.start(_moveUpdateTime);
    _subMoveTimer.start(0);
    _lookTimer.start(0);
    _timeoutTimer.start(0);

    // Start/setup the encoders
    _encoder_L->begin();
    _encoder_R->begin();
    // Start the speed PIDs
    _speedPID_L.begin();
    _speedPID_R.begin();
    _speedPID_L.setOutputLimits(_minPWR, 255.0);
    _speedPID_R.setOutputLimits(_minPWR, 255.0);
    _speedPID_L.setSampleTime(_encoder_L->getSpeedUpdateTime());
    _speedPID_R.setSampleTime(_encoder_R->getSpeedUpdateTime());
    // Start the position PIDs
    _posPID_L.begin();
    _posPID_R.begin();
    _posPID_L.setOutputLimits(-1.0*_posPIDMaxSpeed,_posPIDMaxSpeed);
    _posPID_R.setOutputLimits(-1.0*_posPIDMaxSpeed,_posPIDMaxSpeed);
    _posPID_L.setSampleTime(_encoder_L->getSpeedUpdateTime()*2);
    _posPID_R.setSampleTime(_encoder_R->getSpeedUpdateTime()*2);
}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void MoveManager::updateMove(){
    if(!_is_enabled){return;}

    if(_moveTimer.finished()){
        _moveCompound = random(0,_moveCompoundCount);
        _updateCompoundMove();
    }
}

void MoveManager::updateMove(int8_t inMoveType){
    if(!_is_enabled){return;}

    if(_moveTimer.finished()){
        _moveCompound = inMoveType;
        _updateCompoundMove();
    }
}

//---------------------------------------------------------------------------
// GO
//---------------------------------------------------------------------------
void MoveManager::go(){
    if(!_is_enabled){return;}

    if(_moveCompound == MOVE_C_ZIGZAG){
        zigZag();
    }
    else if(_moveCompound == MOVE_C_SPIRAL){
        spiral();
    }
    else if(_moveCompound == MOVE_C_CIRCLE){
        circle();
    }
    else if(_moveCompound == MOVE_C_LOOK){
        lookAround();
    }
    else{
        forward();
    }
}

//---------------------------------------------------------------------------
// GET,SET and RESET functions: full implementation
//---------------------------------------------------------------------------
void MoveManager::setPWRByDiff(int8_t inDiff){
    _curForwardPWR= _defForwardPWR+inDiff;
    _curBackPWR = _defBackPWR+inDiff;
    _curTurnPWR = _defTurnPWR+inDiff;
  }

void MoveManager::setSpeedByColFlag(bool obstacleClose){
    if(obstacleClose){_speedColFact = _speedColTrue;}
    else{_speedColFact = _speedColFalse;}
    _updateCurrSpeed();
}

void MoveManager::setMoveControl(int8_t inMoveControl){
    if(inMoveControl == MOVE_CONTROL_SPEED){
        // Use PIDs to control speed in mm/s
        _moveControl = MOVE_CONTROL_SPEED;
    }
    else{
        // Bypass PIDs and directly set motor PWM output between 0-255
        _moveControl = MOVE_CONTROL_POWER;
    }
}

void MoveManager::changeCircDir(){
    if(_spiralDirection == MOVE_B_LEFT){
        _spiralDirection = MOVE_B_RIGHT;
    }
    else{
        _spiralDirection = MOVE_B_LEFT;
    }
    if(_circleDirection == MOVE_B_LEFT){
        _circleDirection = MOVE_B_RIGHT;
    }
    else{
        _circleDirection = MOVE_B_LEFT;
    }
}

void MoveManager::setSpeedByMoodFact(float inFact){
_speedMoodFact = inFact;
_updateCurrSpeed();
}

//---------------------------------------------------------------------------
// CALCULATORS
//---------------------------------------------------------------------------
uint16_t MoveManager::calcTimeout(float inSpeed, float inDist){
    float absSpeed = abs(inSpeed);
    float absDist = abs(inDist);
    float timeToVel = absSpeed/_speedTimeoutAccel; // seconds
    float timeToDist = sqrt((2*absDist)/_speedTimeoutAccel); //seconds
    float timeout = 0.0;
    if(timeToDist < timeToVel){ // Finish moving before we finish acceleratin
        timeout = timeToDist;
    }
    else{ // Finish moving after we finish accelerating
        float timeAtConstVel = (absDist-0.5*_speedTimeoutAccel*timeToVel*timeToVel)/absSpeed;
        timeout = timeAtConstVel+timeToVel;
    }
    return uint16_t(_speedTimeoutSF*timeout*1000.0); // milliseconds
}

//---------------------------------------------------------------------------
// BASIC MOVEMENT FUNCTIONS - GENERIC (switched by _moveControl var)
//---------------------------------------------------------------------------

//----------------------------------------------------------------------------
// MoveManager Stop - same regardless of control mode
void MoveManager::stop(){
    _updateBasicMove(MOVE_B_STOP);
    _motorL->run(RELEASE);
    _motorR->run(RELEASE);
    _motorL->setSpeed(0);
    _motorR->setSpeed(0);
}

void MoveManager::stopNoUpdate(){
    _motorL->run(RELEASE);
    _motorR->run(RELEASE);
    _motorL->setSpeed(0);
    _motorR->setSpeed(0);
}

//----------------------------------------------------------------------------
// MoveManager Forward
void MoveManager::forward(){
    if(_moveControl == MOVE_CONTROL_SPEED){
        forwardSPD(_curForwardSpeed);
    }
    else{
        forwardPWR(_curForwardPWR);
    }
}
void MoveManager::forward(float inSpeed){
    forwardSPD(inSpeed);
}
void MoveManager::forward(uint8_t inPower){
    forwardPWR(inPower);
}

//----------------------------------------------------------------------------
// Move Back
void MoveManager::back(){
    if(_moveControl == MOVE_CONTROL_SPEED){
        backSPD(_curBackSpeed);
    }
    else{
        backPWR(_curBackPWR);
    }
}
void MoveManager::back(float inSpeed){
    backSPD(inSpeed);
}
void MoveManager::back(uint8_t inPower){
    backPWR(inPower);
}

//----------------------------------------------------------------------------
// Move Left
void MoveManager::left(){
    if(_moveControl == MOVE_CONTROL_SPEED){
        leftSPD(_curTurnSpeed);
    }
    else{
        leftPWR(_curTurnPWR);
    }
}
void MoveManager::left(float inSpeed){
    leftSPD(inSpeed);
}
void MoveManager::left(uint8_t inPower){
    leftPWR(inPower);
}

//----------------------------------------------------------------------------
// Move Right
void MoveManager::right(){
    if(_moveControl == MOVE_CONTROL_SPEED){
        rightSPD(_curTurnSpeed);
    }
    else{
        rightPWR(_curTurnPWR);
    }
}
void MoveManager::right(float inSpeed){
    rightSPD(inSpeed);
}
void MoveManager::right(uint8_t inPower){
    rightPWR(inPower);
}

//----------------------------------------------------------------------------
// Move Forward Left
void MoveManager::forwardLeft(){
if(_moveControl == MOVE_CONTROL_SPEED){
    forwardLeft(_curForwardSpeed, _curTurnSpeedDiff);
}
else{
    forwardLeft(_curForwardPWR, _curTurnPWRDiff);
}
}

void MoveManager::forwardLeftDiffFrac(float diffFrac){
if(_moveControl == MOVE_CONTROL_SPEED){
    float speedDiff = _curForwardSpeed*diffFrac;
    forwardLeft(_curForwardSpeed, speedDiff);
}
else{
    uint8_t pwrDiff = round(diffFrac*float(_curForwardPWR));
    forwardLeft(_curForwardPWR, pwrDiff);
}
}

void MoveManager::forwardLeft(float inSpeedDiff){
    forwardLeftSPD(_curTurnSpeed,inSpeedDiff);
}
void MoveManager::forwardLeft(float inSpeed, float inSpeedDiff){
    forwardLeftSPD(inSpeed,inSpeedDiff);
}

void MoveManager::forwardLeft(uint8_t inPowerDiff){
    forwardLeftPWR(_curTurnPWR,inPowerDiff);
}
void MoveManager::forwardLeft(uint8_t inPower, uint8_t inPowerDiff){
    forwardLeftPWR(inPower,inPowerDiff);
}

//----------------------------------------------------------------------------
// Move Forward Right
void MoveManager::forwardRight(){
if(_moveControl == MOVE_CONTROL_SPEED){
    forwardRight(_curForwardSpeed, _curTurnSpeedDiff);
}
else{
    forwardRight(_curForwardPWR, _curTurnPWRDiff);
}
}

void MoveManager::forwardRightDiffFrac(float diffFrac){
if(_moveControl == MOVE_CONTROL_SPEED){
    float speedDiff = _curForwardSpeed*diffFrac;
    forwardRight(_curForwardSpeed, speedDiff);
}
else{
    uint8_t pwrDiff = round(diffFrac*float(_curForwardPWR));
    forwardRight(_curForwardPWR, pwrDiff);
}
}

void MoveManager::forwardRight(float inSpeedDiff){
forwardRightSPD(_curTurnSpeed,inSpeedDiff);
}
void MoveManager::forwardRight(float inSpeed, float inSpeedDiff){
forwardRightSPD(inSpeed,inSpeedDiff);
}

void MoveManager::forwardRight(uint8_t inPowerDiff){
forwardRightPWR(_curTurnPWR,inPowerDiff);
}
void MoveManager::forwardRight(uint8_t inPower, uint8_t inPowerDiff){
forwardRightPWR(inPower,inPowerDiff);
}

//----------------------------------------------------------------------------
// Move Circle
void MoveManager::circle(){
if(_moveControl == MOVE_CONTROL_SPEED){
    circleSPD(_curTurnSpeed,_circleDiffSpeed,_circleDirection);
}
else{
    circlePWR(_curTurnPWR,_circleDiffPWR,_circleDirection);
}
}
void MoveManager::circle(int8_t turnDir){
if(_moveControl == MOVE_CONTROL_SPEED){
    circleSPD(_curTurnSpeed,_circleDiffSpeed,turnDir);
}
else{
    circlePWR(_curTurnPWR,_circleDiffPWR,turnDir);
}
}

//============================================================================
// BASIC MOVEMENT FUNCTIONS - CONTROL BY POWER
//============================================================================
// MOVE - POWER CONTROL - SPECIFY POWER
void MoveManager::forwardPWR(uint8_t inPower){
    _updateBasicMove(MOVE_B_FORWARD);
    _motorL->run(FORWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inPower);
    _motorR->setSpeed(inPower);
}

void MoveManager::backPWR(uint8_t inPower){
    _updateBasicMove(MOVE_B_BACK);
    _motorL->run(BACKWARD);
    _motorR->run(BACKWARD);
    _motorL->setSpeed(inPower);
    _motorR->setSpeed(inPower);
}

void MoveManager::leftPWR(uint8_t inPower){
    _updateBasicMove(MOVE_B_LEFT);
    _motorL->run(BACKWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inPower);
    _motorR->setSpeed(inPower);
}

void MoveManager::rightPWR(uint8_t inPower){
    _updateBasicMove(MOVE_B_RIGHT);
    _motorL->run(FORWARD);
    _motorR->run(BACKWARD);
    _motorL->setSpeed(inPower);
    _motorR->setSpeed(inPower);
}

void MoveManager::forwardLeftPWR(uint8_t inPower, uint8_t inPowerDiff){
    _updateBasicMove(MOVE_B_FORLEFT);
    _motorL->run(FORWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inPower-inPowerDiff/2);
    _motorR->setSpeed(inPower+inPowerDiff/2);
}

void MoveManager::forwardRightPWR(uint8_t inPower, uint8_t inPowerDiff){
    _updateBasicMove(MOVE_B_FORRIGHT);
    _motorL->run(FORWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inPower+inPowerDiff/2);
    _motorR->setSpeed(inPower-inPowerDiff/2);
}

void MoveManager::circlePWR(uint8_t inPower, int8_t inPowerDiff, int8_t turnDir){
    if(turnDir == MOVE_B_LEFT){
        forwardLeftPWR(inPower,inPowerDiff);
    }
    else{
        forwardRightPWR(inPower,inPowerDiff);
    }
}

//============================================================================
// BASIC MOVEMENT - SPEED CONTROL with PID
//============================================================================
// MOVE - SPEED CONTROL - SPECIFY SPEED
void MoveManager::forwardSPD(float inSpeed){
    _updateBasicMove(MOVE_B_FORWARD);
    _atSpeed(inSpeed,inSpeed);
}

void MoveManager::backSPD(float inSpeed){
    _updateBasicMove(MOVE_B_BACK);
    _atSpeed(inSpeed,inSpeed);
}

void MoveManager::leftSPD(float inSpeed){
    _updateBasicMove(MOVE_B_LEFT);
    _atSpeed(-1.0*inSpeed,inSpeed);
}

void MoveManager::forwardLeftSPD(float inSpeed, float inSpeedDiff){
    _updateBasicMove(MOVE_B_FORLEFT);
    _atSpeed(inSpeed-(0.5*inSpeedDiff),inSpeed+(0.5*inSpeedDiff));
}

void MoveManager::rightSPD(float inSpeed){
    _updateBasicMove(MOVE_B_RIGHT);
    _atSpeed(inSpeed,-1.0*inSpeed);
}

void MoveManager::forwardRightSPD(float inSpeed, float inSpeedDiff){
    _updateBasicMove(MOVE_B_FORRIGHT);
    _atSpeed(inSpeed+(0.5*inSpeedDiff),inSpeed-(0.5*inSpeedDiff));
}

void MoveManager::circleSPD(float inSpeed, float inSpeedDiff, int8_t turnDir){
    if(turnDir == MOVE_B_LEFT){
        forwardLeft(inSpeed,inSpeedDiff);
    }
    else{
        forwardRight(inSpeed,inSpeedDiff);
    }
}

// NOTE: position can be negative to move backwards
void MoveManager::toDistCtrlPos(float setDist){
    _updateBasicMove(MOVE_B_TODIST_CPOS);
    _toPos(setDist,setDist);
}
void MoveManager::toDistCtrlPos(float setDistL, float setDistR){
    _updateBasicMove(MOVE_B_TODIST_CPOS);
    _toPos(setDistL,setDistR);
}

void MoveManager::turnToAngleCtrlPos(float setAngle){
    _updateBasicMove(MOVE_B_TOANG_CPOS);
    float arcLeng = setAngle*_wheelCircAng;
    _toPos(-1.0*arcLeng,arcLeng);
}

int8_t MoveManager::toDistCtrlSpd(float speedL, float speedR,
    float setDistL, float setDistR){
    int8_t isComplete = 0;
    _updateBasicMove(MOVE_B_TODIST_CSPD);

    // If the set distance changes outside tolerance force update
    if(!((setDistL >= (_toDistSetPtL-_toDistTol)) && (setDistL <= (_toDistSetPtL+_toDistTol)))){
        _toDistSetPtL = setDistL;
        _updateBasicMove(MOVE_B_FORCEUPD);
    }
    if(!((setDistR >= (_toDistSetPtR-_toDistTol)) && (setDistR <= (_toDistSetPtR+_toDistTol)))){
        _toDistSetPtR = setDistR;
        _updateBasicMove(MOVE_B_FORCEUPD);
    }

    // At the start we store our target counts for each encode
    if(_encCountStart){
        uint16_t timeoutL = calcTimeout(speedL,setDistL);
        uint16_t timeoutR = calcTimeout(speedR,setDistR);
        if(timeoutL > timeoutR){
        _timeoutTimer.start(timeoutL);
        }
        else{
        _timeoutTimer.start(timeoutR);
        }

        _encCountStart = false;
        _encCountDiffL = int32_t(setDistL/_encoder_L->getMMPerCount());
        _encCountDiffR = int32_t(setDistR/_encoder_R->getMMPerCount());
        _endEncCountL = _startEncCountL + _encCountDiffL;
        _endEncCountR = _startEncCountR + _encCountDiffR;

        /*
        Serial.print("MMPCount= "); Serial.print(_encoder_L->getMMPerCount());
        Serial.print(",SetDistL= "); Serial.print(setDistL); Serial.print(",SetDistR= "); Serial.print(setDistR);
        Serial.print(",ECDiffL= "); Serial.print(_encCountDiffL); Serial.print(",ECDiffR= "); Serial.print(_encCountDiffR);
        Serial.println();
        Serial.print("StartECount: L="); Serial.print(_startEncCountL); Serial.print(", R="); Serial.print(_startEncCountR);
        Serial.println();
        Serial.print("EndECount: L="); Serial.print(_endEncCountL); Serial.print(", R="); Serial.print(_endEncCountR);
        Serial.println();
        Serial.println();
        */
    }

    if(_timeoutTimer.finished()){
        isComplete = 2;
    }
    else{
        if((setDistL > 0.0) && (setDistR > 0.0)){ // Go forward
        if((_encoder_L->getCount() <= _endEncCountL)||(_encoder_R->getCount() <= _endEncCountR)){
            _atSpeed(abs(speedL),abs(speedR));
        }
        else{
            isComplete = 1;
            stopNoUpdate();
        }
        }
        else if((setDistL < 0.0) && (setDistR > 0.0)){ // Turn left
            if((_encoder_L->getCount() >= _endEncCountL)||(_encoder_R->getCount() <= _endEncCountR)){
                _atSpeed(-1.0*abs(speedL),abs(speedR));
            }
            else{
                isComplete = 1;
                stopNoUpdate();
            }
        }
        else if((setDistL > 0.0) && (setDistR < 0.0)){
            if((_encoder_L->getCount() <= _endEncCountL)||(_encoder_R->getCount() >= _endEncCountR)){
                _atSpeed(abs(speedL),-1.0*abs(speedR));
            }
            else{
                isComplete = 1;
                stopNoUpdate();
            }
        }
        else if((setDistL < 0.0) && (setDistR < 0.0)){ // Turn right
            if((_encoder_L->getCount() >= _endEncCountL)||(_encoder_R->getCount() >= _endEncCountR)){
                _atSpeed(-1.0*abs(speedL),-1.0*abs(speedR));
            }
            else{
                isComplete = 1;
                stopNoUpdate();
            }
        }
        else{
            isComplete = 1;
            stopNoUpdate();
        }
    }
    return isComplete;
}

int8_t MoveManager::toDistCtrlSpd(float setDist){
    int8_t isComplete = 0;
    if(setDist < 0.0){
        isComplete = toDistCtrlSpd(_curBackSpeed,_curBackSpeed,setDist,setDist);
    }
    else{
        isComplete = toDistCtrlSpd(_curForwardSpeed,_curForwardSpeed,setDist,setDist);
    }
    return isComplete;
}

int8_t MoveManager::turnToAngleCtrlSpd(float setAngle){
    float setDist = setAngle*_wheelCircAng;
    int8_t isComplete = 0;
    if(setAngle > 0.0){ // Turn left
        isComplete = toDistCtrlSpd(-1.0*_curTurnSpeed,_curTurnSpeed,-1.0*setDist,setDist);
    }
    else if(setAngle < 0.0){
        isComplete = toDistCtrlSpd(_curTurnSpeed,-1.0*_curTurnSpeed,-1.0*setDist,setDist);
    }
    else{
        isComplete = 1;
    }
    return isComplete;
}

//============================================================================
// COMPOUND MOVEMENT FUNCTIONS
//============================================================================
// NOTE: these two functions already work with the default speed
// ADD: compound move code to each of these

//----------------------------------------------------------------------------
// MOVE WIGGLE LEFT/RIGHT
void MoveManager::wiggle(){
    wiggle(_wiggleDefLeftDur,_wiggleDefRightDur);
}

void MoveManager::wiggle(uint16_t leftDur, uint16_t rightDur){
    // Update the wiggle direction if needed
    if(_subMoveTimer.finished()){
        stop(); // Stop the motors because we are going to switch direction
        if(_wiggleLeftFlag){
        // If we are turning left, switch to right
        _wiggleLeftFlag = false;
        _wiggleCurrDur = rightDur;
        }
        else{
        // If we are turning right, switch to left
        _wiggleLeftFlag = true;
        _wiggleCurrDur = leftDur;
        }
        _subMoveTimer.start(_wiggleCurrDur);
    }

    if(_wiggleLeftFlag){
        left();
    }
    else{
        right();
    }
}

//----------------------------------------------------------------------------
// MOVE FORWARD/BACK
void MoveManager::forwardBack(){
    forwardBack(_FBDefForwardDur,_FBDefBackDur);
}

void MoveManager::forwardBack(uint16_t forwardDur, uint16_t backDur){
    // Update the forward/back direction if needed
    if(_subMoveTimer.finished()){
        stop(); // Stop the motors because we are going to switch direction
        if(_FBForwardFlag){
        // If we are going forward, switch to back
        _FBForwardFlag = false;
        _FBCurrDur = backDur;
        }
        else{
        // If we are going back, switch to forward
        _FBForwardFlag = true;
        _FBCurrDur = forwardDur;
        }
        _subMoveTimer.start(_FBCurrDur);
    }

    if(_FBForwardFlag){
        forward();
    }
    else{
        back();
    }
}

//----------------------------------------------------------------------------
// MOVE SPIRAL
void MoveManager::spiral(){
    spiral(_spiralDirection);
}
void MoveManager::spiral(int8_t turnDir){
    if(_moveControl == MOVE_CONTROL_SPEED){
        spiralSPD(turnDir);
    }
    else{
        spiralPWR(turnDir);
    }
}

void MoveManager::spiralSPD(int8_t turnDir){
    if(_spiralStart || (_moveCompound != MOVE_C_SPIRAL)){
        // Reset the flag so we don't re-init,
        _spiralStart = false;
        _moveCompound = MOVE_C_SPIRAL;
        // Calculate the speed/time slope for the linear increase of speed
        // for the slow wheel
        _initSpiralSpeedDiff = _curForwardSpeed-_spiralMinSpeed;
        _curSpiralSpeedDiff = _initSpiralSpeedDiff;
        _spiralSlope = _initSpiralSpeedDiff/float(_spiralDuration);
        // Start the spiral timer
        _subMoveTimer.start(_spiralDuration);
    }

    // Calculate the current speed for the slope wheel based on the timer
    _spiralCurrTime = _subMoveTimer.getTime();
    _curSpiralSpeedDiff = _initSpiralSpeedDiff - _spiralSlope*float(_spiralCurrTime);

    // Check if we are increasing the speed of the slow wheel above the fast one
    if(_curSpiralSpeedDiff>_curForwardSpeed){
        _curSpiralSpeedDiff = _curForwardSpeed-_minSpeed;
    }

    // If the spiral time is finished then set the flag to restart the spiral
    if(_subMoveTimer.finished()){
        _spiralStart = true;
    }
    else{
        circleSPD(_curForwardSpeed,_curSpiralSpeedDiff,turnDir);
    }
}

void MoveManager::spiralPWR(int8_t turnDir){
    if(_spiralStart || (_moveCompound != MOVE_C_SPIRAL)){
        // Reset the flag so we don't re-init,
        _spiralStart = false;
        _moveCompound = MOVE_C_SPIRAL;
        // Calculate the speed/time slope for the linear increase of speed
        // for the slow wheel
        _initSpiralSpeedDiffPWR = _curForwardPWR-_spiralMinPWR;
        _curSpiralSpeedDiffPWR = _initSpiralSpeedDiffPWR;
        _spiralSlopePWR = float(_initSpiralSpeedDiffPWR)/float(_spiralDuration);
        // Start the spiral timer
        _subMoveTimer.start(_spiralDuration);
    }

    _spiralCurrTime = _subMoveTimer.getTime();
    _curSpiralSpeedDiffPWR = round(float(_initSpiralSpeedDiff) - _spiralSlopePWR*float(_spiralCurrTime));

    if(_curSpiralSpeedDiffPWR>_curForwardPWR){
        _curSpiralSpeedDiffPWR = _curForwardPWR-_spiralMinPWR;
    }

    if(_subMoveTimer.finished()){
        _spiralStart = true;
    }
    else{
        circlePWR(_curForwardPWR,_curSpiralSpeedDiffPWR,turnDir);
    }
}

//----------------------------------------------------------------------------
// MOVE ZIG/ZAG
void MoveManager::zigZag(){
    if(_zzTurnFlag){
        if(!_subMoveTimer.finished()){
        if(_zzTurnDir == MOVE_B_LEFT){
            if(_moveControl == MOVE_CONTROL_SPEED){
                forwardLeft(_curTurnSpeed,_zzTurnDiffSpeed);
            }
            else{
                forwardLeft(_curTurnPWR,_zzTurnDiffPWR);
            }
        }
        else{
            if(_moveControl == MOVE_CONTROL_SPEED){
                forwardRight(_curTurnSpeed,_zzTurnDiffSpeed);
            }
            else{
                forwardRight(_curTurnPWR,_zzTurnDiffPWR);
            }
        }
        }
        else{
        if(_zzTurnDir == MOVE_B_LEFT){
            _zzTurnDir = MOVE_B_RIGHT;
            _zzTurnDuration = _zzRightTurnDur;
        }
        else{
            _zzTurnDir = MOVE_B_LEFT;
            _zzTurnDuration = _zzLeftTurnDur;
        }
        _zzTurnFlag = false;
        _zzStraightFlag = true;
        _subMoveTimer.start(_zzStraightDuration);
        }
    }

    if(_zzStraightFlag){
        if(!_subMoveTimer.finished()){
            forward();
        }
        else{
            _zzTurnFlag = true;
            _zzStraightFlag = false;
            _subMoveTimer.start(_zzTurnDuration);
        }
    }
}

//----------------------------------------------------------------------------
// MOVE LOOK AROUND
void MoveManager::lookAround(){
    if(_lookStartFlag){
        //Serial.println("LOOK START.");
        _moveCompound = MOVE_C_LOOK;
        _lookStartFlag = false;
        _lookTimer.start(_lookMoveTime);
        _lookCurAng = 0;
        _lookMoveSwitch = true;
        resetPIDs();
    }
    else{
        if(_lookTimer.finished()){
        _lookMoveSwitch = !_lookMoveSwitch;
        //Serial.print("LOOK TIMER FINISHED: ");
        if(_lookMoveSwitch){
            //Serial.println("START. Ang++");
            _lookTimer.start(_lookMoveTime);
            _lookCurAng++;

            /*if(_lookCurAng >= _lookNumAngs){
            _lookCurAng = 0;
            }*/
        }
        else{
            //Serial.println("PAUSE.");
            _lookTimer.start(_lookPauseTime);
        }
        }
    }

    if(_lookMoveSwitch && (_lookCurAng <= _lookNumAngs)){
        float moveAng = 0.0;
        if(_lookCurAng == 0){
        moveAng = _lookAngles[_lookCurAng];
        }
        else{
        moveAng = _lookAngles[_lookCurAng] - _lookAngles[_lookCurAng-1];
        }
        turnToAngleCtrlPos(moveAng);
    }
    else{
        stop();
    }
}

void MoveManager::forceLookMove(){
    //Serial.println("FUNC: force look move. Ang++");
    _lookMoveSwitch = true;
    _lookCurAng++;
    _lookTimer.start(_lookMoveTime);
    resetPIDs();
}

void MoveManager::resetLook(){
    //Serial.println("FUNC: reset look.");
    _lookStartFlag = true;
    _lookCurAng = 0;
    resetPIDs();
}

//----------------------------------------------------------------------------
// PIDs - GET/SET FUNCTIONS
//----------------------------------------------------------------------------
void MoveManager::resetPIDs(){
    // Reset Position Control PIDs and Variables
    _setPointRelCounts_L = 0;
    _setPointRelCounts_R = 0;
    _posPID_L.setSetPoint(0.0);
    _posPID_R.setSetPoint(0.0);
    _posPID_L.setOutput(0.0);
    _posPID_R.setOutput(0.0);
    _posPID_L.setControllerOn(PID_OFF);
    _posPID_R.setControllerOn(PID_OFF);
    _posAtL = false;
    _posAtR = false;
    _posAtBoth = false;
    // Reset Speed PIDs
    _speedPID_L.setSetPoint(0.0);
    _speedPID_R.setSetPoint(0.0);
    _speedPID_L.setOutput(0.0);
    _speedPID_R.setOutput(0.0);
    _speedPID_L.setControllerOn(PID_OFF);
    _speedPID_R.setControllerOn(PID_OFF);
}

//----------------------------------------------------------------------------
// PRIVATE HELPER FUNCTIONS
void MoveManager::_updateBasicMove(int8_t inMove){
    if(_moveBasic != inMove){
        _moveBasic = inMove;
        _encCountStart = true;
        _startEncCountL = _encoder_L->getCount();
        _startEncCountR = _encoder_R->getCount();
        resetPIDs();
    }
}

//----------------------------------------------------------------------------
void MoveManager::_updateCompoundMove(){
    _moveUpdateTime = random(_moveUpdateMinTime,_moveUpdateMaxTime);

    if(_moveCompound == MOVE_C_SPIRAL){
        _spiralStart = true;
        _spiralDirection = random(0,2);
        _moveUpdateTime = _spiralDuration;
    }
    else if(_moveCompound == MOVE_C_CIRCLE){
        _circleDirection = random(0,2);
    }
    else if(_moveCompound == MOVE_C_LOOK){
        _lookStartFlag = true;
        _moveUpdateTime = _lookTotTime;
    }

    // Restart timers
    _moveTimer.start(_moveUpdateTime);
    _subMoveTimer.start(0);
}

//----------------------------------------------------------------------------
void MoveManager::_atSpeed(float inSpeedL,float inSpeedR){
    // Check if the left/right PIDs are on if not turn them on
    if(!_speedPID_L.getControllerOn()){
        _speedPID_L.setControllerOn(PID_ON);
        _speedPID_L.setPGainOnly(_speedPRev);
        if(inSpeedL < 0.0){
        _speedPID_L.setControllerDir(PID_REVERSE);

        }
        else{
        _speedPID_L.setControllerDir(PID_DIRECT);
        }
        if((inSpeedL < 0.0) && (inSpeedR < 0.0)){
        _speedPID_R.setPGainOnly(_speedPRev);
        _speedPID_L.setPGainOnly(_speedPRev);
        }
    }
    if(!_speedPID_R.getControllerOn()){
        _speedPID_R.setControllerOn(PID_ON);
        _speedPID_R.setPGainOnly(_speedPRev);
        if(inSpeedR < 0.0){
        _speedPID_R.setControllerDir(PID_REVERSE);
        }
        else{
        _speedPID_R.setControllerDir(PID_DIRECT);
        }
        if((inSpeedL < 0.0) && (inSpeedR < 0.0)){
        _speedPID_R.setPGainOnly(_speedPRev);
        _speedPID_L.setPGainOnly(_speedPRev);
        }
    }

    // Update the set point
    _speedPID_L.setSetPoint(inSpeedL);
    _speedPID_R.setSetPoint(inSpeedR);

    // Update left and right speed PIDs
    _speedPID_L.update(_encoder_L->getSmoothSpeedMMPS());
    _speedPID_R.update(_encoder_R->getSmoothSpeedMMPS());

    // If the speed is negative then set motors to run backward
    if(inSpeedL < 0.0){
        _motorL->run(BACKWARD);
    }
    else{
        _motorL->run(FORWARD);
    }
    if(inSpeedR < 0.0){
        _motorR->run(BACKWARD);
    }
    else{
        _motorR->run(FORWARD);
    }
    _motorL->setSpeed(int(_speedPID_L.getOutput()));
    _motorR->setSpeed(int(_speedPID_R.getOutput()));
}

//----------------------------------------------------------------------------
void MoveManager::_toPos(float setPosL, float setPosR){
    // LEFT
    if(!_posPID_L.getControllerOn()){
        _posPID_L.setControllerOn(PID_ON);
        _speedPID_L.setControllerOn(PID_ON);
        //_speedPID_L.setPIDGains(_speedP,_speedI,_speedD);
    }
    // RIGHT
    if(!_posPID_R.getControllerOn()){
        _posPID_R.setControllerOn(PID_ON);
        _speedPID_R.setControllerOn(PID_ON);
        //_speedPID_R.setPIDGains(_speedP,_speedI,_speedD);
    }

    // Check if the set point passed to the function has changed
    // LEFT
    int32_t checkSetPointL =  round(setPosL/_encoder_L->getMMPerCount());
    if(checkSetPointL != _setPointRelCounts_L){
        _setPointRelCounts_L = checkSetPointL;
        _startEncCount_L = _encoder_L->getCount();
        _posPID_L.setSetPoint(float(_setPointRelCounts_L));
    }
    // RIGHT
    int32_t checkSetPointR =  round(setPosR/_encoder_R->getMMPerCount());
    if(checkSetPointR != _setPointRelCounts_R){
        _setPointRelCounts_R = checkSetPointR;
        _startEncCount_R = _encoder_R->getCount();
        _posPID_R.setSetPoint(float(_setPointRelCounts_R));
    }

    // Update the relative count and send it to the PIDs
    // LEFT
    _currRelCount_L = _encoder_L->getCount()-_startEncCount_L;
    _posPID_L.update(_currRelCount_L);
    // RIGHT
    _currRelCount_R = _encoder_R->getCount()-_startEncCount_R;
    _posPID_R.update(_currRelCount_R);

    // Update the speed PIDs
    // LEFT
    _speedPID_L.update(_encoder_L->getSmoothSpeedMMPS());
    // RIGHT
    _speedPID_R.update(_encoder_R->getSmoothSpeedMMPS());

    // Check that the PID is sending a signal above the min speed
    // LEFT
    if(round(abs(_posPID_L.getOutput())) < _posPIDMinSpeed){
        if(_posPID_L.getOutput() < 0.0){
        _posPID_L.setOutput(-1.0*_posPIDMinSpeed);
        }
        else{
        _posPID_L.setOutput(_posPIDMinSpeed);
        }
    }
    // RIGHT
    if(round(abs(_posPID_R.getOutput())) < _posPIDMinSpeed){
        if(_posPID_R.getOutput() < 0.0){
        _posPID_R.setOutput(-1.0*_posPIDMinSpeed);
        }
        else{
        _posPID_R.setOutput(_posPIDMinSpeed);
        }
    }

    // Move forward or back based on the PID value
    // LEFT
    if(_currRelCount_L < (_setPointRelCounts_L-_posTol)){
        _speedPID_L.setSetPoint(_posPID_L.getOutput());
        _speedPID_L.setControllerDir(PID_DIRECT);
        _motorL->run(FORWARD);
        _motorL->setSpeed(int(_speedPID_L.getOutput()));
    }
    else if(_currRelCount_L > (_setPointRelCounts_L+_posTol)){
        _speedPID_L.setSetPoint(_posPID_L.getOutput());
        _speedPID_L.setControllerDir(PID_REVERSE);
        _motorL->run(BACKWARD);
        _motorL->setSpeed(int(_speedPID_L.getOutput()));
    }
    else{
        _posPID_L.setOutput(0.0);
        _speedPID_L.setOutput(0.0);
        _speedPID_L.setSetPoint(0.0);
        _motorL->run(RELEASE);
        _motorL->setSpeed(0);
        _posAtL = true;
    }
    // RIGHT
    if(_currRelCount_R < (_setPointRelCounts_R-_posTol)){
        _speedPID_R.setSetPoint(_posPID_R.getOutput());
        _speedPID_R.setControllerDir(PID_DIRECT);
        _motorR->run(FORWARD);
        _motorR->setSpeed(int(_speedPID_R.getOutput()));
    }
    else if(_currRelCount_R > (_setPointRelCounts_R+_posTol)){
        _speedPID_R.setSetPoint(_posPID_R.getOutput());
        _speedPID_R.setControllerDir(PID_REVERSE);
        _motorR->run(BACKWARD);
        _motorR->setSpeed(int(_speedPID_R.getOutput()));
    }
    else{
        _posPID_R.setOutput(0.0);
        _speedPID_R.setOutput(0.0);
        _speedPID_R.setSetPoint(0.0);
        _motorR->run(RELEASE);
        _motorR->setSpeed(0);
        _posAtR = true;
    }

    if(_posAtL && _posAtR){
        _posAtBoth = true;
    }
}

//----------------------------------------------------------------------------
void MoveManager::_updateCurrSpeed(){
    _curForwardSpeed = constrain(_defForwardSpeed*_speedMoodFact*_speedColFact,_minSpeed,_maxSpeed);
    _curBackSpeed = -1.0*constrain(fabs(_defBackSpeed*_speedMoodFact*_speedColFact),_minSpeed,_maxSpeed);
    _curTurnSpeed = constrain(_defTurnSpeed*_speedMoodFact*_speedColFact,_minSpeed,_maxSpeed);
}
