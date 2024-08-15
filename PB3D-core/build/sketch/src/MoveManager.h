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
#define MOVE_B_TODIST_CSpeed 10
#define MOVE_B_TOANG_CPOS 11
#define MOVE_B_TOANG_CSpeed 12

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
  void update_move();
  void update_move(int8_t inMoveType);

  //---------------------------------------------------------------------------
  // GO: Called during explore or other task to randomise movements
  //---------------------------------------------------------------------------
   void go();

  //---------------------------------------------------------------------------
  // GET,SET and RESET functions: Inline
  //---------------------------------------------------------------------------
  // MOVE TYPE - Get/Set
  int8_t get_basic_move(){return _move_basic;}
  int8_t get_compound_move(){return _move_compound;}
  void set_compound_move(int8_t inMoveCode){_move_compound = inMoveCode;}


  // MOTOR POWER CONTROL - Get/Set
  uint8_t get_min_power(){return _min_power;}
  uint8_t get_max_power(){return _max_power;}

  uint8_t get_forward_power(){return _cur_forward_power;}
  uint8_t get_back_power(){return _cur_back_power;}
  uint8_t get_turn_power(){return _cur_turn_power;}

  void set_forward_power(uint8_t inPower){_cur_forward_power = inPower;}
  void set_back_power(uint8_t inPower){_cur_back_power = inPower;}
  void set_turn_power(uint8_t inPower){_cur_turn_power = inPower;}

  // MOTOR SPEED CONTROL - Get/Set
  float get_min_speed(){return _min_speed;}
  float get_max_speed(){return _max_speed;}

  float get_forward_speed(){return _cur_forward_speed;}
  float get_back_speed(){return _cur_back_speed;}
  float get_turn_speed(){return _cur_turn_speed;}

  void set_forward_speed(float inSpeed){_cur_forward_speed = fabs(inSpeed);}
  void set_back_speed(float inSpeed){_cur_back_speed = -1.0*fabs(inSpeed);}
  void set_turn_speed(float inSpeed){_cur_turn_speed = fabs(inSpeed);}

  // ENCODERS - Get/Set
  int32_t get_encoder_count_left(){return _encoder_left->get_count();}
  int32_t get_encoder_count_right(){return _encoder_right->get_count();}
  float get_encoder_speed_left(){return _encoder_left->get_smooth_speed_mmps();}
  float get_encoder_speed_right(){return _encoder_right->get_smooth_speed_mmps();}
  float get_encoder_mm_per_count(){return _encoder_left->get_mm_per_count();}

  // MOVE TIMERS - Reset
  void reset_move_timer(){_move_timer.start(0);}
  void reset_submove_timer(){_submove_timer.start(0);}

  //---------------------------------------------------------------------------
  // GET,SET and RESET functions: full implementation
  //---------------------------------------------------------------------------
  void set_power_by_diff(int8_t inDiff);
  void set_speed_by_col_code(bool obstacleClose);
  void set_move_control(int8_t inMoveControl);
  void change_circ_dir();
  void set_speed_by_mood_fact(float inFact);

  //---------------------------------------------------------------------------
  // CALCULATORS
  //---------------------------------------------------------------------------
  uint16_t calc_timeout(float inSpeed, float inDist);

  //---------------------------------------------------------------------------
  // BASIC MOVEMENT FUNCTIONS - GENERIC (switched by _moveControl var)
  //---------------------------------------------------------------------------

  //----------------------------------------------------------------------------
  // MoveManager Stop - same regardless of control mode
  void stop();
  void stop_no_update();

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
  void forward_left();
  void forward_left_diff_frac(float diffFrac);
  void forward_left(float inSpeedDiff);
  void forward_left(float inSpeed, float inSpeedDiff);
  void forward_left(uint8_t inPowerDiff);
  void forward_left(uint8_t inPower, uint8_t inPowerDiff);

  //----------------------------------------------------------------------------
  // Move Forward Right
  void forward_right();
  void forward_right_diff_frac(float diffFrac);
  void forward_right(float inSpeedDiff);
  void forward_right(float inSpeed, float inSpeedDiff);
  void forward_right(uint8_t inPowerDiff);
  void forward_right(uint8_t inPower, uint8_t inPowerDiff);

  //----------------------------------------------------------------------------
  // Move Circle
  void circle();
  void circle(int8_t turnDir);

  //============================================================================
  // BASIC MOVEMENT FUNCTIONS - CONTROL BY POWER
  //============================================================================
   // MOVE - POWER CONTROL - SPECIFY POWER
  void forward_power(uint8_t inPower);
  void back_power(uint8_t inPower);
  void left_power(uint8_t inPower);
  void right_power(uint8_t inPower);
  void forward_left_power(uint8_t inPower, uint8_t inPowerDiff);
  void forward_right_power(uint8_t inPower, uint8_t inPowerDiff);
  void circle_power(uint8_t inPower, int8_t inPowerDiff, int8_t turnDir);

  //============================================================================
  // BASIC MOVEMENT - SPEED CONTROL with PID
  //============================================================================
  // MOVE - SPEED CONTROL - SPECIFY SPEED
  void forward_speed(float inSpeed);
  void back_speed(float inSpeed);
  void left_speed(float inSpeed);
  void forward_left_speed(float inSpeed, float inSpeedDiff);
  void right_speed(float inSpeed);
  void forward_right_speed(float inSpeed, float inSpeedDiff);
  void circle_speed(float inSpeed, float inSpeedDiff, int8_t turnDir);
  // NOTE: position can be negative to move backwards
  void to_dist_ctrl_pos(float setDist);
  void to_dist_ctrl_pos(float setDistL, float setDistR);
  void turn_to_angle_ctrl_pos(float setAngle);
  int8_t to_dist_ctrl_speed(float speedL, float speedR, float setDistL, float setDistR);
  int8_t to_dist_ctrl_speed(float setDist);
  int8_t turn_to_angle_ctrl_speed(float setAngle);

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
  void forward_back();
  void forward_back(uint16_t forwardDur, uint16_t backDur);

  //----------------------------------------------------------------------------
  // MOVE SPIRAL
  void spiral();
  void spiral(int8_t turnDir);
  void spiral_speed(int8_t turnDir);
  void spiral_power(int8_t turnDir);

  //----------------------------------------------------------------------------
  // MOVE ZIG/ZAG
  void zig_zag();

  //----------------------------------------------------------------------------
  // MOVE LOOK AROUND
  void look_around();
  void force_look_move();
  void reset_look();

  // LOOK - Diagnostics
  bool look_is_moving(){return _look_move_switch;}
  bool look_is_paused(){return !_look_move_switch;}

  // LOOK - Getters
  bool get_look_move_switch(){return _look_move_switch;}
  bool get_look_finished(){return (_look_cur_ang >= _look_num_angs);}
  uint8_t get_look_curr_ang_ind(){return _look_cur_ang;}
  uint8_t get_look_num_angs(){return _look_num_angs;}
  uint8_t get_look_max_angs(){return _look_max_angs;}
  float get_look_ang_from_ind(uint8_t inInd){return _look_angles[inInd];}
  uint16_t get_look_move_time(){return _look_move_time;}
  uint16_t get_look_pause_time(){return _look_pause_time;}
  uint16_t get_look_tot_time(){return _look_tot_time;}


  //----------------------------------------------------------------------------
  // PIDs - GET/SET FUNCTIONS
  //----------------------------------------------------------------------------
  void reset_PIDs();

  //----------------------------------------------------------------------------
  // Position PID get functions - left/right motors
  int32_t get_pos_count_left(){return _curr_relative_count_left;}
  float get_pos_PID_set_point_left(){return _pos_PID_left.get_set_point();}
  float get_pos_PID_output_left(){return _pos_PID_left.get_output();}

  int32_t get_pos_count_right(){return _curr_relative_count_right;}
  float get_pos_PID_set_point_right(){return _pos_PID_right.get_set_point();}
  float get_pos_PID_output_right(){return _pos_PID_right.get_output();}

  bool get_pos_PID_attained_set_point(){return _pos_at_both;}

  //----------------------------------------------------------------------------
  // Speed PID get functions - left/right motors
  float getSpeedPIDSetPoint_L(){return _speedPID_L.get_set_point();}
  float getSpeedPIDOutput_L(){return _speedPID_L.get_output();}
  float getSpeedPIDP_L(){return _speedPID_L.getPropTerm();}
  float getSpeedPIDI_L(){return _speedPID_L.getIntTerm();}
  float getSpeedPIDD_L(){return _speedPID_L.getDerivTerm();}

  float getSpeedPIDSetPoint_R(){return _speedPID_R.get_set_point();}
  float getSpeedPIDOutput_R(){return _speedPID_R.get_output();}
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
  Encoder* _encoder_left = NULL;
  Encoder* _encoder_right = NULL;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Type and General Variables
  bool _is_enabled = true;

  int8_t _moveControl = MOVE_CONTROL_SPEED;
  int8_t _move_basic = MOVE_B_FORWARD;
  int8_t _move_compound = MOVE_C_STRAIGHT;
  int8_t _moveCompoundCount = MOVE_C_COUNT;
  uint32_t _moveUpdateTime = 5000;
  uint32_t _moveUpdateMinTime = 4000;
  uint32_t _moveUpdateMaxTime = 12000;
  Timer _move_timer = Timer();
  Timer _submove_timer = Timer();
  Timer _timeoutTimer = Timer();

  // Encoder counter variables
  bool _encCountStart = true;
  int32_t _startEncCountL = 0, _startEncCountR = 0;
  int32_t _endEncCountL = 0, _endEncCountR = 0;
  int32_t _encCountDiffL = 0, _encCountDiffR = 0;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Motor Power (Speed) Variables
  uint8_t _defForwardPower = 120;
  uint8_t _defBackPower = 120;
  uint8_t _defTurnPower = 100;
  uint8_t _defTurnPowerDiff = 80;

  uint8_t _cur_forward_power = _defForwardPower;
  uint8_t _cur_back_power = _defBackPower;
  uint8_t _cur_turn_power = _defTurnPower;
  uint8_t _curTurnPowerDiff = _defTurnPowerDiff;

  uint8_t _min_power = 25;
  uint8_t _max_power = 255;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Motor Speed Variables in mm/s (millimeters per second)
  float _defForwardSpeed = 350.0;
  float _defBackSpeed = -225.0;
  float _defTurnSpeed = 250.0;
  float _defTurnSpeedDiff = 0.75*_defTurnSpeed;

  float _cur_forward_speed = _defForwardSpeed;
  float _cur_back_speed = _defBackSpeed;
  float _cur_turn_speed = _defTurnSpeed;
  float _curTurnSpeedDiff = _defTurnSpeedDiff;

  float _speedMoodFact = 1.0;
  float _speedColFact = 1.0;
  float _speedColTrue = 0.8, _speedColFalse = 1.0;
  float _min_speed = 50.0, _max_speed = 1000.0;

  // Estimating power for given speed - updated for new wheels - 24th Sept 2022
  // NOTE: turned speed estimation off because PID has less overshoot without
  // Updated again 5th Jan 2023 - RF wood floor tests - slope=0.166,int=22.7
  // Used to be set with an offset of 50.0 and slope 0.0
  float _speedToPowerSlope = 0.166, _speedToPowerOffset = 22.7, _speedToPowerMin = 22.7;
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
  PID _pos_PID_left = PID(false,0.6,0.0,0.0,20);
  PID _pos_PID_right = PID(false,0.6,0.0,0.0,20);
  float _posPIDMinSpeed = 100.0, _posPIDMaxSpeed = 200.0;
  int16_t _posTol = 3;
  int32_t _startEncCount_L = 0, _setPointRelCounts_L = 0, _curr_relative_count_left = 0;
  int32_t _startEncCount_R = 0, _setPointRelCounts_R = 0, _curr_relative_count_right = 0;

  float _wheelBase = 172.0; // UPDATED: 1st Jan 2023 - new stable geom chassis with large wheels
  float _wheel_circ = _wheelBase*PI, _wheelCircAng = (_wheelBase*PI)/(360.0); // FIXED factor of 2 by adding encode interrupt
  bool _posAtL = false, _posAtR = false, _pos_at_both = false;
  // NOTE: D(inner) = 122mm, D(outer) = 160mm, D(avg) = 141mm

  //----------------------------------------------------------------------------
  // MOVE OBJ - To Dist/Angle
  float _toDistSetPtL = 0.0, _toDistSetPtR = 0.0;
  float _toDistTol = 5.0;
  float _toAngSetPt = 0.0;
  float _toAngTol = 1.0;

  //----------------------------------------------------------------------------
  // MOVE OBJ - Circle Variables
  uint8_t _circleDiffPower = 30;
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

  uint8_t _zzTurnDiffPower = round(_defForwardPower/2);
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
  float _initSpiralSpeedDiff = _defForwardSpeed-_min_speed;
  float _curSpiralSpeedDiff = 0.0;

  uint8_t _spiralMinPower = 5.0;
  float _spiralSlopePower = 0.0;
  uint8_t _initSpiralSpeedDiffPower = _defForwardPower-_min_power;
  uint8_t _curSpiralSpeedDiffPower = 0;

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
  uint16_t _look_move_time = 2200;
  uint16_t _look_pause_time = 500;
  bool _look_move_switch = false;
  uint8_t _look_cur_ang = 0;
  uint8_t _look_num_angs = 4;
  uint8_t _look_max_angs = 8;
  float _look_angles[8] = {30,0,-30,0,0,0,0,0};
  uint16_t _look_tot_time = _look_num_angs*(_look_move_time+_look_pause_time);
};
#endif // MOVE_H
