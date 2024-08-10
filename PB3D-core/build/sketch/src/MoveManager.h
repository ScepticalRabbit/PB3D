#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/MoveManager.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

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

class MoveManager{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  //---------------------------------------------------------------------------
  MoveManager(Adafruit_MotorShield* AFMS, Encoder* encL, Encoder* encR);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  //---------------------------------------------------------------------------
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: Called during LOOP
  //---------------------------------------------------------------------------
  void updateMove();
  void updateMove(int8_t inMoveType);

  //---------------------------------------------------------------------------
  // GO: Called during explore or other task to randomise movements
  //---------------------------------------------------------------------------
   void go();

  //---------------------------------------------------------------------------
  // GET,SET and RESET functions: Inline
  //---------------------------------------------------------------------------
  // MOVE TYPE - Get/Set
  int8_t getBasicMove(){return _moveBasic;}
  int8_t getCompoundMove(){return _moveCompound;}
  void setCompoundMove(int8_t inMoveCode){_moveCompound = inMoveCode;}


  // MOTOR POWER CONTROL - Get/Set
  uint8_t getMinPower(){return _minPWR;}
  uint8_t getMaxPower(){return _maxPWR;}

  uint8_t getForwardPWR(){return _curForwardPWR;}
  uint8_t getBackPWR(){return _curBackPWR;}
  uint8_t getTurnPWR(){return _curTurnPWR;}

  void setForwardPWR(uint8_t inPower){_curForwardPWR = inPower;}
  void setBackPWR(uint8_t inPower){_curBackPWR = inPower;}
  void setTurnPWR(uint8_t inPower){_curTurnPWR = inPower;}

  // MOTOR SPEED CONTROL - Get/Set
  float getMinSpeed(){return _minSpeed;}
  float getMaxSpeed(){return _maxSpeed;}

  float getForwardSpeed(){return _curForwardSpeed;}
  float getBackSpeed(){return _curBackSpeed;}
  float getTurnSpeed(){return _curTurnSpeed;}

  void setForwardSpeed(float inSpeed){_curForwardSpeed = fabs(inSpeed);}
  void setBackSpeed(float inSpeed){_curBackSpeed = -1.0*fabs(inSpeed);}
  void setTurnSpeed(float inSpeed){_curTurnSpeed = fabs(inSpeed);}

  // ENCODERS - Get/Set
  int32_t getEncCountL(){return _encoder_L->getCount();}
  int32_t getEncCountR(){return _encoder_R->getCount();}
  float getEncSpeedL(){return _encoder_L->getSmoothSpeedMMPS();}
  float getEncSpeedR(){return _encoder_R->getSmoothSpeedMMPS();}
  float getEncMMPerCount(){return _encoder_L->getMMPerCount();}

  // MOVE TIMERS - Reset
  void resetMoveTimer(){_moveTimer.start(0);}
  void resetSubMoveTimer(){_subMoveTimer.start(0);}

  //---------------------------------------------------------------------------
  // GET,SET and RESET functions: full implementation
  //---------------------------------------------------------------------------
  void setPWRByDiff(int8_t inDiff);
  void setSpeedByColFlag(bool obstacleClose);
  void setMoveControl(int8_t inMoveControl);
  void changeCircDir();
  void setSpeedByMoodFact(float inFact);

  //---------------------------------------------------------------------------
  // CALCULATORS
  //---------------------------------------------------------------------------
  uint16_t calcTimeout(float inSpeed, float inDist);

  //---------------------------------------------------------------------------
  // BASIC MOVEMENT FUNCTIONS - GENERIC (switched by _moveControl var)
  //---------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // MoveManager Stop - same regardless of control mode
  void stop();
  void stopNoUpdate();

  //----------------------------------------------------------------------------
  // MoveManager Forward
  void forward();
  void forward(float inSpeed);
  void forward(uint8_t inPower);

  //----------------------------------------------------------------------------
  // Move Back
  void back();
  void back(float inSpeed);
  void back(uint8_t inPower);

  //----------------------------------------------------------------------------
  // Move Left
  void left();
  void left(float inSpeed);
  void left(uint8_t inPower);

  //----------------------------------------------------------------------------
  // Move Right
  void right();
  void right(float inSpeed);
  void right(uint8_t inPower);

  //----------------------------------------------------------------------------
  // Move Forward Left
  void forwardLeft();
  void forwardLeftDiffFrac(float diffFrac);
  void forwardLeft(float inSpeedDiff);
  void forwardLeft(float inSpeed, float inSpeedDiff);
  void forwardLeft(uint8_t inPowerDiff);
  void forwardLeft(uint8_t inPower, uint8_t inPowerDiff);

  //----------------------------------------------------------------------------
  // Move Forward Right
  void forwardRight();
  void forwardRightDiffFrac(float diffFrac);
  void forwardRight(float inSpeedDiff);
  void forwardRight(float inSpeed, float inSpeedDiff);
  void forwardRight(uint8_t inPowerDiff);
  void forwardRight(uint8_t inPower, uint8_t inPowerDiff);

  //----------------------------------------------------------------------------
  // Move Circle
  void circle();
  void circle(int8_t turnDir);

  //============================================================================
  // BASIC MOVEMENT FUNCTIONS - CONTROL BY POWER
  //============================================================================
   // MOVE - POWER CONTROL - SPECIFY POWER
  void forwardPWR(uint8_t inPower);
  void backPWR(uint8_t inPower);
  void leftPWR(uint8_t inPower);
  void rightPWR(uint8_t inPower);
  void forwardLeftPWR(uint8_t inPower, uint8_t inPowerDiff);
  void forwardRightPWR(uint8_t inPower, uint8_t inPowerDiff);
  void circlePWR(uint8_t inPower, int8_t inPowerDiff, int8_t turnDir);

  //============================================================================
  // BASIC MOVEMENT - SPEED CONTROL with PID
  //============================================================================
  // MOVE - SPEED CONTROL - SPECIFY SPEED
  void forwardSPD(float inSpeed);
  void backSPD(float inSpeed);
  void leftSPD(float inSpeed);
  void forwardLeftSPD(float inSpeed, float inSpeedDiff);
  void rightSPD(float inSpeed);
  void forwardRightSPD(float inSpeed, float inSpeedDiff);
  void circleSPD(float inSpeed, float inSpeedDiff, int8_t turnDir);
  // NOTE: position can be negative to move backwards
  void toDistCtrlPos(float setDist);
  void toDistCtrlPos(float setDistL, float setDistR);
  void turnToAngleCtrlPos(float setAngle);
  int8_t toDistCtrlSpd(float speedL, float speedR, float setDistL, float setDistR);
  int8_t toDistCtrlSpd(float setDist);
  int8_t turnToAngleCtrlSpd(float setAngle);

  //============================================================================
  // COMPOUND MOVEMENT FUNCTIONS
  //============================================================================
  // NOTE: these two functions already work with the default speed
  // ADD: compound move code to each of these

  //----------------------------------------------------------------------------
  // MOVE WIGGLE LEFT/RIGHT
  void wiggle();
  void wiggle(uint16_t leftDur, uint16_t rightDur);

  //----------------------------------------------------------------------------
  // MOVE FORWARD/BACK
  void forwardBack();
  void forwardBack(uint16_t forwardDur, uint16_t backDur);

  //----------------------------------------------------------------------------
  // MOVE SPIRAL
  void spiral();
  void spiral(int8_t turnDir);
  void spiralSPD(int8_t turnDir);
  void spiralPWR(int8_t turnDir);

  //----------------------------------------------------------------------------
  // MOVE ZIG/ZAG
  void zigZag();

  //----------------------------------------------------------------------------
  // MOVE LOOK AROUND
  void lookAround();
  void forceLookMove();
  void resetLook();

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


  //----------------------------------------------------------------------------
  // PIDs - GET/SET FUNCTIONS
  //----------------------------------------------------------------------------
  void resetPIDs();

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
  // PRIVATE HELPER FUNCTIONS
  void _updateBasicMove(int8_t inMove);
  void _updateCompoundMove();
  void _atSpeed(float inSpeedL,float inSpeedR);
  void _toPos(float setPosL, float setPosR);
  void _updateCurrSpeed();

  //----------------------------------------------------------------------------
  // MOVE OBJ - Pointers to external objects

  // Adafruit Motor Shield Objects
  Adafruit_MotorShield* _AFMS = NULL;
  Adafruit_DCMotor* _motorL = NULL;
  Adafruit_DCMotor* _motorR = NULL;

  // Encoder Objects
  Encoder* _encoder_L = NULL;
  Encoder* _encoder_R = NULL;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Type and General Variables
  bool _isEnabled = true;

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
