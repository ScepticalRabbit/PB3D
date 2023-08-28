//----------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS - MOVE
//----------------------------------------------------------------------------
/*
The task class is part of the PetBot (PB) program. It used to...

Author: Lloyd Fletcher
Date Created: 28th Aug. 2021
Date Edited:  28th Aug. 2021
*/

#ifndef MOVE_H
#define MOVE_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "Encoder.h"
#include "Timer.h"
#include "PID.h"

// MOVEMENT CONTROL 
#define MOVE_CONTROL_POWER 0
#define MOVE_CONTROL_SPEED 1

// MOVEMENT escape codes
#define MOVE_E_RANDDIR -1
#define MOVE_E_NOREV -2

// BASIC movement codes
#define MOVE_B_STOP -10
#define MOVE_B_FORCEUPD -1
#define MOVE_B_FORWARD 0
#define MOVE_B_BACK 1
#define MOVE_B_LEFT 3
#define MOVE_B_RIGHT 4
#define MOVE_B_FORLEFT 5
#define MOVE_B_FORRIGHT 6
#define MOVE_B_BACKLEFT 7
#define MOVE_B_BACKRIGHT 8
#define MOVE_B_TODIST_CPOS 9
#define MOVE_B_TODIST_CSPD 10
#define MOVE_B_TOANG_CPOS 11
#define MOVE_B_TOANG_CSPD 12

// COMPOUND movement codes
#define MOVE_C_ESCAPE -1
#define MOVE_C_STRAIGHT 0
#define MOVE_C_ZIGZAG 1
#define MOVE_C_CIRCLE 2
#define MOVE_C_SPIRAL 3
#define MOVE_C_LOOK 4
// Increment with last code above +1
#define MOVE_C_COUNT 5 

class Move{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  Move(Adafruit_MotorShield* AFMS, Encoder* encL, Encoder* encR){
      _AFMS = AFMS;
      _encoder_L = encL;
      _encoder_R = encR;
  }
  
  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop  
  void begin(){
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
  // UPDATE
  void updateMove(){
    if(_moveTimer.finished()){
      _moveCompound = random(0,_moveCompoundCount);
      _updateCompoundMove();
    }
  }

  void updateMove(int8_t inMoveType){
    if(_moveTimer.finished()){
      _moveCompound = inMoveType;
      _updateCompoundMove();
    }
  }

  //---------------------------------------------------------------------------
  // GO
  void go(){
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

  //============================================================================
  // MOVE OBJ - Type/Speed GET,SET and RESET functions
  //============================================================================
  // MOVE TYPE - Get/Set
  int8_t getBasicMove(){return _moveBasic;}
  int8_t getCompoundMove(){return _moveCompound;}
  void setCompoundMove(int8_t inMoveCode){_moveCompound = inMoveCode;}

  void setMoveControl(int8_t inMoveControl){
    if(inMoveControl == MOVE_CONTROL_SPEED){
      // Use PIDs to control speed in mm/s
      _moveControl = MOVE_CONTROL_SPEED;
    }
    else{
      // Bypass PIDs and directly set motor PWM output between 0-255
      _moveControl = MOVE_CONTROL_POWER;
    }
  }

  void changeCircDir(){
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

  // MOTOR POWER CONTROL - Get/Set
  uint8_t getMinPower(){return _minPWR;}
  uint8_t getMaxPower(){return _maxPWR;}
  
  uint8_t getForwardPWR(){return _curForwardPWR;}
  uint8_t getBackPWR(){return _curBackPWR;}
  uint8_t getTurnPWR(){return _curTurnPWR;}

  void setForwardPWR(uint8_t inPower){_curForwardPWR = inPower;}
  void setBackPWR(uint8_t inPower){_curBackPWR = inPower;}
  void setTurnPWR(uint8_t inPower){_curTurnPWR = inPower;}
  
  void setPWRByDiff(int8_t inDiff){
    _curForwardPWR= _defForwardPWR+inDiff;
    _curBackPWR = _defBackPWR+inDiff;
    _curTurnPWR = _defTurnPWR+inDiff;  
  }

  // MOTOR SPEED CONTROL - Get/Set
  float getMinSpeed(){return _minSpeed;}
  float getMaxSpeed(){return _maxSpeed;}
  
  float getForwardSpeed(){return _curForwardSpeed;}
  float getBackSpeed(){return _curBackSpeed;}
  float getTurnSpeed(){return _curTurnSpeed;}
  
  void setForwardSpeed(float inSpeed){_curForwardSpeed = fabs(inSpeed);}
  void setBackSpeed(float inSpeed){_curBackSpeed = -1.0*fabs(inSpeed);}
  void setTurnSpeed(float inSpeed){_curTurnSpeed = fabs(inSpeed);}

  void setSpeedByColFlag(bool obstacleClose){
    if(obstacleClose){_speedColFact = _speedColTrue;}
    else{_speedColFact = _speedColFalse;}
    _updateCurrSpeed();    
  }

  void setSpeedByMoodFact(float inFact){
    _speedMoodFact = inFact;
    _updateCurrSpeed();
  }

  // ENCODERS - Get/Set
  int32_t getEncCountL(){return _encoder_L->getCount();}
  int32_t getEncCountR(){return _encoder_R->getCount();}
  float getEncSpeedL(){return _encoder_L->getSmoothSpeedMMPS();}
  float getEncSpeedR(){return _encoder_R->getSmoothSpeedMMPS();}
  float getEncMMPerCount(){return _encoder_L->getMMPerCount();}

  // MOVE TIMERS - Reset
  void resetMoveTimer(){_moveTimer.start(0);}
  void resetSubMoveTimer(){_subMoveTimer.start(0);}

  // CALCULATORS
  uint16_t calcTimeout(float inSpeed, float inDist){
    float absSpeed = abs(inSpeed);
    float absDist = abs(inDist);
    float timeToVel = absSpeed/_speedTimeoutAccel; // seconds
    float timeToDist = sqrt((2*absDist)/_speedTimeoutAccel); //seconds
    float timeout = 0.0;
    if(timeToDist < timeToVel){ // Finish moving before we finish acceleratin
      timeout = timeToDist;
    }
    else{ // Finish moving after we finish accelerating
      //float distWhileAccel = 0.5*_speedTimeoutAccel*timeToVel*timeToVel; // s=1/2*a*t^2
      //float distRemaining = inDist-distWhileAccel;
      //float timeAtConstVel = distRemaining/inSpeed;
      float timeAtConstVel = (absDist-0.5*_speedTimeoutAccel*timeToVel*timeToVel)/absSpeed;
      timeout = timeAtConstVel+timeToVel;
    }
    return uint16_t(_speedTimeoutSF*timeout*1000.0); // milliseconds
  }
  
  //============================================================================
  // BASIC MOVEMENT FUNCTIONS - GENERIC (switched by _moveControl var)
  //============================================================================
  
  //----------------------------------------------------------------------------
  // Move Stop - same regardless of control mode
  void stop(){
    _updateBasicMove(MOVE_B_STOP);
    _motorL->run(RELEASE);
    _motorR->run(RELEASE);
    _motorL->setSpeed(0);
    _motorR->setSpeed(0);
  }

  void stopNoUpdate(){
    _motorL->run(RELEASE);
    _motorR->run(RELEASE);
    _motorL->setSpeed(0);
    _motorR->setSpeed(0);
  }

  //----------------------------------------------------------------------------
  // Move Forward 
  void forward(){
    if(_moveControl == MOVE_CONTROL_SPEED){
      forwardSPD(_curForwardSpeed);
    }
    else{
      forwardPWR(_curForwardPWR);
    }
  }
  void forward(float inSpeed){
    forwardSPD(inSpeed);
  }
  void forward(uint8_t inPower){
    forwardPWR(inPower);
  }

  //----------------------------------------------------------------------------
  // Move Back
  void back(){
    if(_moveControl == MOVE_CONTROL_SPEED){
      backSPD(_curBackSpeed);
    }
    else{
      backPWR(_curBackPWR);
    }
  }
  void back(float inSpeed){
    backSPD(inSpeed);
  }
  void back(uint8_t inPower){
    backPWR(inPower);
  }

  //----------------------------------------------------------------------------
  // Move Left
  void left(){
    if(_moveControl == MOVE_CONTROL_SPEED){
      leftSPD(_curTurnSpeed);
    }
    else{
      leftPWR(_curTurnPWR);
    }
  }
  void left(float inSpeed){
    leftSPD(inSpeed);
  }
  void left(uint8_t inPower){
    leftPWR(inPower);
  }

  //----------------------------------------------------------------------------
  // Move Right
  void right(){
    if(_moveControl == MOVE_CONTROL_SPEED){
      rightSPD(_curTurnSpeed);
    }
    else{
      rightPWR(_curTurnPWR);
    }
  }
  void right(float inSpeed){
    rightSPD(inSpeed);
  }
  void right(uint8_t inPower){
    rightPWR(inPower);
  }
  
  //----------------------------------------------------------------------------
  // Move Forward Left
  void forwardLeft(){
    if(_moveControl == MOVE_CONTROL_SPEED){
      forwardLeft(_curForwardSpeed, _curTurnSpeedDiff);
    }
    else{
      forwardLeft(_curForwardPWR, _curTurnPWRDiff);
    }
  }

  void forwardLeftDiffFrac(float diffFrac){
    if(_moveControl == MOVE_CONTROL_SPEED){
      float speedDiff = _curForwardSpeed*diffFrac;
      forwardLeft(_curForwardSpeed, speedDiff);
    }
    else{
      uint8_t pwrDiff = round(diffFrac*float(_curForwardPWR));
      forwardLeft(_curForwardPWR, pwrDiff);
    }
  }
  
  void forwardLeft(float inSpeedDiff){
    forwardLeftSPD(_curTurnSpeed,inSpeedDiff);
  }
  void forwardLeft(float inSpeed, float inSpeedDiff){
    forwardLeftSPD(inSpeed,inSpeedDiff);
  }
  
  void forwardLeft(uint8_t inPowerDiff){
    forwardLeftPWR(_curTurnPWR,inPowerDiff);
  }
  void forwardLeft(uint8_t inPower, uint8_t inPowerDiff){
    forwardLeftPWR(inPower,inPowerDiff);
  }

  //----------------------------------------------------------------------------
  // Move Forward Right
  void forwardRight(){
    if(_moveControl == MOVE_CONTROL_SPEED){
      forwardRight(_curForwardSpeed, _curTurnSpeedDiff);
    }
    else{
      forwardRight(_curForwardPWR, _curTurnPWRDiff);
    }
  }

  void forwardRightDiffFrac(float diffFrac){
    if(_moveControl == MOVE_CONTROL_SPEED){
      float speedDiff = _curForwardSpeed*diffFrac;
      forwardRight(_curForwardSpeed, speedDiff);
    }
    else{
      uint8_t pwrDiff = round(diffFrac*float(_curForwardPWR));
      forwardRight(_curForwardPWR, pwrDiff);
    }
  }
  
  void forwardRight(float inSpeedDiff){
    forwardRightSPD(_curTurnSpeed,inSpeedDiff);
  }
  void forwardRight(float inSpeed, float inSpeedDiff){
    forwardRightSPD(inSpeed,inSpeedDiff);
  }
  
  void forwardRight(uint8_t inPowerDiff){
    forwardRightPWR(_curTurnPWR,inPowerDiff);
  }
  void forwardRight(uint8_t inPower, uint8_t inPowerDiff){
    forwardRightPWR(inPower,inPowerDiff);
  }

  //----------------------------------------------------------------------------
  // Move Circle
  void circle(){
    if(_moveControl == MOVE_CONTROL_SPEED){
      circleSPD(_curTurnSpeed,_circleDiffSpeed,_circleDirection);
    }
    else{
      circlePWR(_curTurnPWR,_circleDiffPWR,_circleDirection);
    }
  }
  void circle(int8_t turnDir){
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
  void forwardPWR(uint8_t inPower){
    _updateBasicMove(MOVE_B_FORWARD);
    _motorL->run(FORWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inPower);
    _motorR->setSpeed(inPower);
  }
  void backPWR(uint8_t inPower){
    _updateBasicMove(MOVE_B_BACK);
    _motorL->run(BACKWARD);
    _motorR->run(BACKWARD);
    _motorL->setSpeed(inPower);
    _motorR->setSpeed(inPower);
  }
  void leftPWR(uint8_t inPower){
    _updateBasicMove(MOVE_B_LEFT);
    _motorL->run(BACKWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inPower);
    _motorR->setSpeed(inPower);  
  }
  void rightPWR(uint8_t inPower){
    _updateBasicMove(MOVE_B_RIGHT);
    _motorL->run(FORWARD);
    _motorR->run(BACKWARD);
    _motorL->setSpeed(inPower);
    _motorR->setSpeed(inPower);
  }
  void forwardLeftPWR(uint8_t inPower, uint8_t inPowerDiff){
    _updateBasicMove(MOVE_B_FORLEFT);
    _motorL->run(FORWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inPower-inPowerDiff/2);
    _motorR->setSpeed(inPower+inPowerDiff/2);
  }
  void forwardRightPWR(uint8_t inPower, uint8_t inPowerDiff){
    _updateBasicMove(MOVE_B_FORRIGHT);
    _motorL->run(FORWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inPower+inPowerDiff/2);
    _motorR->setSpeed(inPower-inPowerDiff/2);
  }
  void circlePWR(uint8_t inPower, int8_t inPowerDiff, int8_t turnDir){
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
  void forwardSPD(float inSpeed){
    _updateBasicMove(MOVE_B_FORWARD);
    _atSpeed(inSpeed,inSpeed);  
  }
  void backSPD(float inSpeed){
    _updateBasicMove(MOVE_B_BACK);
    _atSpeed(inSpeed,inSpeed);
  }
  
  void leftSPD(float inSpeed){
    _updateBasicMove(MOVE_B_LEFT);
    _atSpeed(-1.0*inSpeed,inSpeed);  
  }
  void forwardLeftSPD(float inSpeed, float inSpeedDiff){
    _updateBasicMove(MOVE_B_FORLEFT);
    _atSpeed(inSpeed-(0.5*inSpeedDiff),inSpeed+(0.5*inSpeedDiff));  
  }
  
  void rightSPD(float inSpeed){
    _updateBasicMove(MOVE_B_RIGHT);
    _atSpeed(inSpeed,-1.0*inSpeed);  
  }
  void forwardRightSPD(float inSpeed, float inSpeedDiff){
    _updateBasicMove(MOVE_B_FORRIGHT);
    _atSpeed(inSpeed+(0.5*inSpeedDiff),inSpeed-(0.5*inSpeedDiff));  
  }
  
  void circleSPD(float inSpeed, float inSpeedDiff, int8_t turnDir){
    if(turnDir == MOVE_B_LEFT){
      forwardLeft(inSpeed,inSpeedDiff);
    }
    else{
      forwardRight(inSpeed,inSpeedDiff);
    }
  }

  // NOTE: position can be negative to move backwards
  void toDistCtrlPos(float setDist){
    _updateBasicMove(MOVE_B_TODIST_CPOS);
    _toPos(setDist,setDist);  
  }
  void toDistCtrlPos(float setDistL, float setDistR){
    _updateBasicMove(MOVE_B_TODIST_CPOS);
    _toPos(setDistL,setDistR);  
  }

  void turnToAngleCtrlPos(float setAngle){
    _updateBasicMove(MOVE_B_TOANG_CPOS);
    float arcLeng = setAngle*_wheelCircAng;
    _toPos(-1.0*arcLeng,arcLeng);
  }
  
  int8_t toDistCtrlSpd(float speedL, float speedR, float setDistL, float setDistR){
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

  int8_t toDistCtrlSpd(float setDist){
    int8_t isComplete = 0;
    if(setDist < 0.0){
      isComplete = toDistCtrlSpd(_curBackSpeed,_curBackSpeed,setDist,setDist);
    }
    else{
      isComplete = toDistCtrlSpd(_curForwardSpeed,_curForwardSpeed,setDist,setDist);
    }
    return isComplete;
  }

  int8_t turnToAngleCtrlSpd(float setAngle){
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
  void wiggle(){
    wiggle(_wiggleDefLeftDur,_wiggleDefRightDur);
  }
  void wiggle(uint16_t leftDur, uint16_t rightDur){
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
  void forwardBack(){
    forwardBack(_FBDefForwardDur,_FBDefBackDur);
  }
  void forwardBack(uint16_t forwardDur, uint16_t backDur){
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
  void spiral(){
    spiral(_spiralDirection);
  } 
  void spiral(int8_t turnDir){
    if(_moveControl == MOVE_CONTROL_SPEED){
      spiralSPD(turnDir);
    }
    else{
      spiralPWR(turnDir);
    }
  } 
  void spiralSPD(int8_t turnDir){
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
  void spiralPWR(int8_t turnDir){
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
  void zigZag(){
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
  void lookAround(){
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
  
  void forceLookMove(){
    //Serial.println("FUNC: force look move. Ang++");
    _lookMoveSwitch = true;
    _lookCurAng++;
    _lookTimer.start(_lookMoveTime);
    resetPIDs();
  }

  void resetLook(){
    //Serial.println("FUNC: reset look.");
    _lookStartFlag = true;
    _lookCurAng = 0;
    resetPIDs();  
  }

  // LOOK - Diagnostics
  bool lookIsMoving(){return _lookMoveSwitch;}
  bool lookIsPaused(){return !_lookMoveSwitch;}

  // LOOK - Getters
  bool getLookMoveSwitch(){return _lookMoveSwitch;}
  bool getLookFinished(){return (_lookCurAng >= _lookNumAngs);}
  uint8_t getLookCurrAngInd(){return _lookCurAng;}
  uint8_t getLookNumAngs(){return _lookNumAngs;}
  uint8_t getLookMaxAngs(){return _lookMaxAngs;}
  float getLookAngFromInd(uint8_t inInd){return _lookAngles[inInd];}
  uint16_t getLookMoveTime(){return _lookMoveTime;}
  uint16_t getLookPauseTime(){return _lookPauseTime;}
  uint16_t getLookTotTime(){return _lookTotTime;}

  
  //============================================================================
  // PIDs - GET/SET FUNCTIONS
  //============================================================================
  // Reset the 
  void resetPIDs(){
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
  // Position PID get functions - left/right motors
  int32_t getPosCount_L(){return _currRelCount_L;}
  float getPosPIDSetPoint_L(){return _posPID_L.getSetPoint();}
  float getPosPIDOutput_L(){return _posPID_L.getOutput();}

  int32_t getPosCount_R(){return _currRelCount_R;}
  float getPosPIDSetPoint_R(){return _posPID_R.getSetPoint();}
  float getPosPIDOutput_R(){return _posPID_R.getOutput();}

  bool getPosPIDAttainSP_Both(){return _posAtBoth;}

  //----------------------------------------------------------------------------
  // Speed PID get functions - left/right motors
  float getSpeedPIDSetPoint_L(){return _speedPID_L.getSetPoint();}  
  float getSpeedPIDOutput_L(){return _speedPID_L.getOutput();}
  float getSpeedPIDP_L(){return _speedPID_L.getPropTerm();}
  float getSpeedPIDI_L(){return _speedPID_L.getIntTerm();}
  float getSpeedPIDD_L(){return _speedPID_L.getDerivTerm();}
          
  float getSpeedPIDSetPoint_R(){return _speedPID_R.getSetPoint();}
  float getSpeedPIDOutput_R(){return _speedPID_R.getOutput();}
  float getSpeedPIDP_R(){return _speedPID_R.getPropTerm();}
  float getSpeedPIDI_R(){return _speedPID_R.getIntTerm();}
  float getSpeedPIDD_R(){return _speedPID_R.getDerivTerm();}
  
private:
  //----------------------------------------------------------------------------
  void _updateBasicMove(int8_t inMove){
    if(_moveBasic != inMove){
      _moveBasic = inMove;
      _encCountStart = true;
      _startEncCountL = _encoder_L->getCount();
      _startEncCountR = _encoder_R->getCount();      
      resetPIDs();
    }
  }

  //----------------------------------------------------------------------------
  void _updateCompoundMove(){
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
  void _atSpeed(float inSpeedL,float inSpeedR){
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
  void _toPos(float setPosL, float setPosR){
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

  void _updateCurrSpeed(){
    _curForwardSpeed = constrain(_defForwardSpeed*_speedMoodFact*_speedColFact,_minSpeed,_maxSpeed);
    _curBackSpeed = -1.0*constrain(fabs(_defBackSpeed*_speedMoodFact*_speedColFact),_minSpeed,_maxSpeed);
    _curTurnSpeed = constrain(_defTurnSpeed*_speedMoodFact*_speedColFact,_minSpeed,_maxSpeed);
  }

  //----------------------------------------------------------------------------
  // MOVE OBJ - Pointers to external objects
  
  // Adafruit Motor Shield Objects 
  Adafruit_MotorShield* _AFMS;
  Adafruit_DCMotor* _motorL;
  Adafruit_DCMotor* _motorR;
  
  // Encoder Objects
  Encoder* _encoder_L;
  Encoder* _encoder_R;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Type and General Variables
  int8_t _moveControl = MOVE_CONTROL_SPEED;  
  int8_t _moveBasic = MOVE_B_FORWARD;
  int8_t _moveCompound = MOVE_C_STRAIGHT;
  int8_t _moveCompoundCount = MOVE_C_COUNT;
  uint32_t _moveUpdateTime = 5000;
  uint32_t _moveUpdateMinTime = 4000;
  uint32_t _moveUpdateMaxTime = 12000;
  Timer _moveTimer = Timer();
  Timer _subMoveTimer = Timer();
  Timer _timeoutTimer = Timer();

  // Encoder counter variables
  bool _encCountStart = true;
  int32_t _startEncCountL = 0, _startEncCountR = 0;
  int32_t _endEncCountL = 0, _endEncCountR = 0;
  int32_t _encCountDiffL = 0, _encCountDiffR = 0;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Motor Power (Speed) Variables
  uint8_t _defForwardPWR = 120;
  uint8_t _defBackPWR = 120;
  uint8_t _defTurnPWR = 100;
  uint8_t _defTurnPWRDiff = 80;
  
  uint8_t _curForwardPWR = _defForwardPWR;
  uint8_t _curBackPWR = _defBackPWR;
  uint8_t _curTurnPWR = _defTurnPWR;
  uint8_t _curTurnPWRDiff = _defTurnPWRDiff;
  
  uint8_t _minPWR = 25;
  uint8_t _maxPWR = 255;
  
  //----------------------------------------------------------------------------
  // MOVE OBJ - Motor Speed Variables in mm/s (millimeters per second)
  float _defForwardSpeed = 350.0;
  float _defBackSpeed = -225.0;
  float _defTurnSpeed = 250.0;
  float _defTurnSpeedDiff = 0.75*_defTurnSpeed;
  
  float _curForwardSpeed = _defForwardSpeed;
  float _curBackSpeed = _defBackSpeed;
  float _curTurnSpeed = _defTurnSpeed;
  float _curTurnSpeedDiff = _defTurnSpeedDiff;

  float _speedMoodFact = 1.0;
  float _speedColFact = 1.0; 
  float _speedColTrue = 0.8, _speedColFalse = 1.0;
  float _minSpeed = 50.0, _maxSpeed = 1000.0;

  // Estimating power for given speed - updated for new wheels - 24th Sept 2022
  // NOTE: turned speed estimation off because PID has less overshoot without
  // Updated again 5th Jan 2023 - RF wood floor tests - slope=0.166,int=22.7
  // Used to be set with an offset of 50.0 and slope 0.0
  float _speedToPWRSlope = 0.166, _speedToPWROffset = 22.7, _speedToPWRMin = 22.7;
  float _speedTimeoutAccel = 1220.0,  _speedTimeoutSF = 2.0;   

  //----------------------------------------------------------------------------
  // MOVE OBJ - Control PIDs
  // TODO: 1st Jan 2023 - need to update gains based on new encoder counts, halved gain as temporary fix (was 0.02)
  //double _speedP = 2.5, _speedI = 0.0, _speedD = 0.0; // NOTE: 2.5 causes osc with 50ms period
  //double _speedP = 0.45*2.5, _speedI = 50.0e-3/1.2, _speedD = 0.0; // Ziegler-Nichols tuning [0.45,/1.2,0.0]
  double _speedP = 0.6*0.8, _speedI = 0.5*50.0e-3, _speedD = 50.0e-3/8.0; // Ziegler-Nichols tuning [0.6,0.5,/8]
  double _speedPRev = _speedP*0.9; // NOTE: turned off setting gains! Caused problems
  PID _speedPID_L = PID(false,_speedP,_speedI,_speedD,10);
  PID _speedPID_R = PID(false,_speedP,_speedI,_speedD,10);

  // Position Control PIDs and Variables
  PID _posPID_L = PID(false,0.6,0.0,0.0,20);
  PID _posPID_R = PID(false,0.6,0.0,0.0,20);
  float _posPIDMinSpeed = 100.0, _posPIDMaxSpeed = 200.0;
  int16_t _posTol = 3; 
  int32_t _startEncCount_L = 0, _setPointRelCounts_L = 0, _currRelCount_L = 0;
  int32_t _startEncCount_R = 0, _setPointRelCounts_R = 0, _currRelCount_R = 0;
  
  float _wheelBase = 172.0; // UPDATED: 1st Jan 2023 - new stable geom chassis with large wheels
  float _wheelCirc = _wheelBase*PI, _wheelCircAng = (_wheelBase*PI)/(360.0); // FIXED factor of 2 by adding encode interrupt
  bool _posAtL = false, _posAtR = false, _posAtBoth = false;
  // NOTE: D(inner) = 122mm, D(outer) = 160mm, D(avg) = 141mm
  
  //----------------------------------------------------------------------------
  // MOVE OBJ - To Dist/Angle
  float _toDistSetPtL = 0.0, _toDistSetPtR = 0.0; 
  float _toDistTol = 5.0;  
  float _toAngSetPt = 0.0;
  float _toAngTol = 1.0;       

  //----------------------------------------------------------------------------
  // MOVE OBJ - Circle Variables
  uint8_t _circleDiffPWR = 30;
  float _circleDiffSpeed = _defForwardSpeed*0.5;
  int8_t _circleDirection = MOVE_B_LEFT;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Zig Zag Variables
  bool _zzTurnFlag = true;
  uint16_t _zzInitTurnDur = 800;
  uint16_t _zzLeftTurnDur = _zzInitTurnDur;
  uint16_t _zzRightTurnDur = _zzInitTurnDur;
  uint16_t _zzTurnDuration = _zzLeftTurnDur;
  
  bool _zzStraightFlag = false;
  uint32_t _zzStraightDuration = 1000;
  int8_t _zzTurnDir = MOVE_B_LEFT;

  uint8_t _zzTurnDiffPWR = round(_defForwardPWR/2);
  float _zzTurnDiffSpeed = 0.5*_defForwardSpeed;
  
  //----------------------------------------------------------------------------
  // MOVE OBJ - Spiral Variables
  bool _spiralStart = true;
  uint32_t _spiralStartTime = 0;
  uint32_t _spiralDuration = 20000;
  uint32_t _spiralCurrTime = 0;
  int8_t _spiralDirection = MOVE_B_LEFT;

  float _spiralMinSpeed = 5.0;
  float _spiralSlope = 0.0;
  float _initSpiralSpeedDiff = _defForwardSpeed-_minSpeed;
  float _curSpiralSpeedDiff = 0.0;

  uint8_t _spiralMinPWR = 5.0;
  float _spiralSlopePWR = 0.0;
  uint8_t _initSpiralSpeedDiffPWR = _defForwardPWR-_minPWR;
  uint8_t _curSpiralSpeedDiffPWR = 0;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Wiggle Variables
  bool _wiggleLeftFlag = true;
  uint16_t _wiggleDefLeftDur = 600;
  uint16_t _wiggleDefRightDur = 600;
  uint16_t _wiggleCurrDur = _wiggleDefLeftDur;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Forward/Back Variables
  bool _FBForwardFlag = true;
  uint16_t _FBDefForwardDur = 500;
  uint16_t _FBDefBackDur = 500;
  uint16_t _FBCurrDur = _FBDefForwardDur; 

  //----------------------------------------------------------------------------
  // MOVE OBJ - Look Around
  bool _lookStartFlag = true;
  Timer _lookTimer = Timer();
  uint16_t _lookMoveTime = 2200;
  uint16_t _lookPauseTime = 500;
  bool _lookMoveSwitch = false;
  uint8_t _lookCurAng = 0;
  uint8_t _lookNumAngs = 4;
  uint8_t _lookMaxAngs = 8;
  float _lookAngles[8] = {30,0,-30,0,0,0,0,0};
  uint16_t _lookTotTime = _lookNumAngs*(_lookMoveTime+_lookPauseTime);
};
#endif // MOVE_H
