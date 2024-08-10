#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/CollisionManager.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------
#ifndef COLLISIONMANAGER_H
#define COLLISIONMANAGER_H

#include <Arduino.h>
#include "MoodManager.h"
#include "MoveManager.h"
#include "TaskManager.h"
#include "Timer.h"

// Helper classes
#include "CollisionDangerCodes.h"
#include "CollisionEscaper.h"
#include "LaserManager.h"
#include "BumperSensor.h"
#include "UltrasonicSensor.h"

//------------------------------------------------------------------------------
// DEFINITIONS
// Address for digital out
#ifndef ADDR_FOLLBOARD
  #define ADDR_FOLLBOARD 0x11
#endif

// DEBUG Flags
#define COLL_DEBUG_DECISIONTREE

//------------------------------------------------------------------------------
// LAST COLLISION: data structure
struct lastCollision_t{
  uint8_t checkVec[7] = {0,0,0,0,0,0,0};
  int16_t USRange = 0;
  int16_t LSRRangeL = 0,LSRRangeR = 0;
  int16_t LSRRangeU = 0,LSRRangeD = 0;
  int8_t LSRStatusL = 0,LSRStatusR = 0;
  int8_t LSRStatusU = 0,LSRStatusD = 0;
  uint8_t escCount = 0;
  float escDist = 0.0, escAng = 0.0;
};

//------------------------------------------------------------------------------
// COLLISION MANAGER: class to handle object avoidance behaviour
class CollisionManager{
public:
  //----------------------------------------------------------------------------
  // CONSTRUCTOR: pass in pointers to main objects and other sensors
  CollisionManager(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove);

  //----------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //----------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //----------------------------------------------------------------------------
  // Get, set and reset
  bool get_enabled_flag(){return _is_enabled;}
  void set_enabled_flag(bool inFlag){_is_enabled = inFlag;}

  bool getDetectFlag(){return _collisionDetected;}

  uint16_t getCount(){return _collisionCount;}
  void incCount(){_collisionCount++;}
  void resetCount(){_collisionCount = 0;}

  bool getBeepBeepFlag(){return _collisionBeepBeepFlag;}
  void setBeepBeepFlag(bool inFlag){_collisionBeepBeepFlag = inFlag;}

  bool getBumperFlag(){return _bumpers.get_bump_flag();}

  int16_t getUSRange(){return _ultrasonicRanger.getRange();}
  int16_t getUSRangeMM(){return _ultrasonicRanger.getRangeMM();}

  int16_t getLSRRange(ELaserIndex _ind){return _laserManager.get_range(_ind);}
  lastCollision_t* getLastCollision(){return &_lastCol;}

  uint8_t getColCheck(uint8_t pos){return _checkVec[pos];}

  bool getAltFlag();
  void resetFlags();

  //----------------------------------------------------------------------------
  // Command forwarding to escaper
  void setEscapeStart();
  void escape();
  bool getEscapeFlag();
  int8_t getEscapeTurn();

private:
  //----------------------------------------------------------------------------
  // Check all ranges and escape decision tree
  void _updateCheckVec();
  void _updateEscapeDecision(); // Escape decision tree

  //----------------------------------------------------------------------------
  // CLASS VARIABLES
  // Main object and sensor pointers
  MoodManager* _moodObj = NULL;
  TaskManager* _taskObj = NULL;
  MoveManager* _moveObj = NULL;

  // Collision management variables
  bool _is_enabled = true;
  bool _collisionDetected = false; // Key flag controlling collision escape
  bool _collisionSlowDown = false;

  uint16_t _halfBodyLengMM = 80;
  bool _collisionBeepBeepFlag = false;
  uint16_t _collisionCount = 0;

  // Helper Objects
  CollisionEscaper _escaper = CollisionEscaper();
  LaserManager _laserManager = LaserManager();
  UltrasonicSensor _ultrasonicRanger = UltrasonicSensor();
  BumperSensor _bumpers = BumperSensor();

  // Check flags for all collision sensors
  uint8_t _checkNum = 7;
  uint8_t _checkVec[7] = {0,0,0,0,0,0,0}; //_checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
  uint16_t _checkAllInt = 50;
  Timer _checkAllTimer = Timer();

  // Time to slow down if sensor tripped
  uint16_t _slowDownInt = 500;
  Timer _slowDownTimer = Timer();

  // Collision Sensor Timers
  int16_t _bumperUpdateTime = 101;
  Timer _bumperTimer = Timer();
  int16_t _ultrasonicUpdateTime = 101;  // ms, set to prime number (100+timeout)
  Timer _ultrasonicTimer = Timer();

  // Data structure for info on last collision
  lastCollision_t _lastCol;
};
#endif // COLLISIONMANAGER_H
