//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: 
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
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
// BEGIN: called during SETUP
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
// UPDATE: called during LOOP
//---------------------------------------------------------------------------
void MoveManager::updateMove(){
    if(!_isEnabled){return;}

    if(_moveTimer.finished()){
        _moveCompound = random(0,_moveCompoundCount);
        _updateCompoundMove();
    }
}

void MoveManager::updateMove(int8_t inMoveType){
    if(!_isEnabled){return;}

    if(_moveTimer.finished()){
        _moveCompound = inMoveType;
        _updateCompoundMove();
    }
}

//---------------------------------------------------------------------------
// GO
//---------------------------------------------------------------------------
void MoveManager::go(){
    if(!_isEnabled){return;}

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