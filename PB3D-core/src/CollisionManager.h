//---------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: CollisionManager
//---------------------------------------------------------------------------
/*
The collision manager class is part of the PetBot (PB) program. It handles 
all obstacle avoidance behaviours and the sensors used to detect obstacles.

Author: Lloyd Fletcher
*/
#ifndef COLLISIONMANAGER_H
#define COLLISIONMANAGER_H

#include <Arduino.h>
#include "Ultrasonic.h"
#include "Adafruit_VL53L0X.h"
#include "Mood.h"
#include "Move.h"
#include "Task.h"
#include "Timer.h"

// Helper classes
#include "CollisionDangerFlags.h"
#include "CollisionEscaper.h"
#include "LaserManager.h"

//-----------------------------------------------------------------------------
// DEFINITIONS
//-----------------------------------------------------------------------------
// Collision flags and sub-board address
#define ADDR_NERVSYS 9 
#define COLL_USSENS 7

// Define addresses for all laser sensors
#define ADDR_LSR_L 0x31
#define ADDR_LSR_R 0x32
#define ADDR_LSR_A 0x33
#define ADDR_LSR_U 0x34
#define ADDR_LSR_D 0x35

// Address for digital out 
#ifndef ADDR_FOLLBOARD
  #define ADDR_FOLLBOARD 0x11
#endif

// DEBUG Flags
//#define COLL_DEBUG_DECISIONTREE

//-----------------------------------------------------------------------------
// LAST COLLISION: data structure
//-----------------------------------------------------------------------------
struct lastCollision_t{
  uint8_t checkVec[7] = {0,0,0,0,0,0,0}; 
  uint16_t USRange = 0;
  uint16_t LSRRangeL = 0,LSRRangeR = 0;
  uint16_t LSRRangeU = 0,LSRRangeD = 0;
  uint8_t escCount = 0; 
  float escDist = 0.0, escAng = 0.0;
};

//-----------------------------------------------------------------------------
// COLLISION MANAGER: class to handle object avoidance behaviour
//-----------------------------------------------------------------------------
class CollisionManager{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR: pass in pointers to main objects and other sensors
  //---------------------------------------------------------------------------
  CollisionManager(Mood* inMood, Task* inTask, Move* inMove, Ultrasonic* ultrasonicSens);

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP 
  //---------------------------------------------------------------------------
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during LOOP 
  //---------------------------------------------------------------------------
  void update();

  //---------------------------------------------------------------------------
  // Get, set and reset
  //--------------------------------------------------------------------------- 
  bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

  uint16_t getCount(){return _collisionCount;}
  void incCount(){_collisionCount++;}
  void resetCount(){_collisionCount = 0;}
  
  bool getBeepBeepFlag(){return _collisionBeepBeepFlag;}
  void setBeepBeepFlag(bool inFlag){_collisionBeepBeepFlag = inFlag;}

  bool getDetectFlag(){return _collisionDetected;} 
  bool getBumperFlag(){return _collisionBumperFlag;}
  // bool getColLSRFlagL(){return _collisionLSRFlagL;}
  // bool getColLSRFlagR(){return _collisionLSRFlagR;}
  // bool getColLSRFlagB(){return _collisionLSRFlagB;}
  // bool getColLSRFlagU(){return _collisionLSRFlagU;}
  // bool getColLSRFlagD(){return _collisionLSRFlagD;}
  
  bool getColUSFlag(){return _collisionUSFlag;}
  

  int16_t getUSRange(){return _USSensRange;}
  int16_t getUSRangeMM(){return (_USSensRange*10);}
  int16_t getLSRRangeL(){return _laserManager.getRangeL();}
  int16_t getLSRRangeR(){return _laserManager.getRangeR();}
  int16_t getLSRRangeA(){return _laserManager.getRangeA();}
  int16_t getLSRRangeU(){return _laserManager.getRangeU();}
  int16_t getLSRRangeD(){return _laserManager.getRangeD();}
  lastCollision_t* getLastCollision(){return &_lastCol;}

  bool getAltFlag();
  void resetFlags();

  //---------------------------------------------------------------------------
  // Command forwarding to escaper
  //---------------------------------------------------------------------------
  void setEscapeStart();
  void escape();
  bool getEscapeFlag();
  int8_t getEscapeTurn();

private:
  //---------------------------------------------------------------------------
  // Check collision detection sensors
  //---------------------------------------------------------------------------
  void _updateUSRanger();
  void _updateBumpers();
  // void _updateColLSRs();
  // void _updateAltLSR();
  // void _updateUpDownLSRs();

  //---------------------------------------------------------------------------
  // Check all ranges and escape decision tree
  //---------------------------------------------------------------------------
  void _updateCheckVec();
  void _updateEscapeDecision(); // Escape decision tree

  //---------------------------------------------------------------------------
  // LASER INIT: set all I2C addresses
  //---------------------------------------------------------------------------
  // void _initLSR(byte sendByte, Adafruit_VL53L0X* LSRObj, bool* LSROn,
  //               uint8_t LSRAddr,char LSRStr);
  // void _setLSRAddrs();

  //---------------------------------------------------------------------------
  // Helper functions
  //---------------------------------------------------------------------------
  // void _sendByteWithI2C(byte inByte);

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  //---------------------------------------------------------------------------
  // Main object and sensor pointers
  Mood* _moodObj = NULL;
  Task* _taskObj = NULL;
  Move* _moveObj = NULL;
  Ultrasonic* _ultrasonicSens = NULL;
  
  // Collision management variables
  bool _isEnabled = true;
  bool _collisionDetected = false; // Key flag controlling collision escape
  bool _collisionSlowDown = false;

  uint16_t _halfBodyLengMM = 80;

  bool _collisionUSFlag = false;  
  byte _collisionNervSys = B00000000;
  bool _collisionBumperFlag = false;

  bool _collisionBeepBeepFlag = false;
  uint16_t _collisionCount = 0;

  // Helper Objects
  CollisionEscaper _escaper;
  LaserManager _laserManager = LaserManager();

  // Check flags for all collision sensors
  uint8_t _checkNum = 7;
  uint8_t _checkVec[7] = {0,0,0,0,0,0,0}; //_checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
  uint16_t _checkAllInt = 51;
  Timer _checkAllTimer = Timer();

  // Time to slow down if sensor tripped
  uint16_t _slowDownInt = 500;
  Timer _slowDownTimer = Timer();

  // Bumper variables
  int8_t _BMPRCount = 0, _BMPRThres = 13;
  int16_t _BMPRUpdateTime = 101;
  Timer _BMPRTimer = Timer();
  
  // Ultrasonic ranger variables
  int16_t _USSensRange = 1000;
  int16_t _USColDistClose = _halfBodyLengMM/10; // cm
  int16_t _USColDistFar = 2*_halfBodyLengMM/10;  // cm  
  int16_t _USColDistSlowD = 4*_halfBodyLengMM/10; // cm
  int16_t _USColDistLim = 4;    // cm 
  int16_t _USUpdateTime = 101;  // ms, set to prime number (100+timeout)
  Timer _USTimer = Timer();

  // Data structure for info on last collision
  lastCollision_t _lastCol;
};
#endif // COLLISIONMANAGER_H
