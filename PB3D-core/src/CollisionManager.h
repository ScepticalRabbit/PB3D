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
#include "CollisionEscaper.h"

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

  bool getDetectFlag(){return _collisionFlag;} 
  bool getBumperFlag(){return _collisionBumperFlag;}
  bool getColLSRFlagL(){return _collisionLSRFlagL;}
  bool getColLSRFlagR(){return _collisionLSRFlagR;}
  bool getColLSRFlagB(){return _collisionLSRFlagB;}
  bool getColLSRFlagU(){return _collisionLSRFlagU;}
  bool getColLSRFlagD(){return _collisionLSRFlagD;}
  bool getAltFlag(){return _collisionLSRFlagB;}
  bool getColUSFlag(){return _collisionUSFlag;}
    
  int16_t getUSRange(){return _USSensRange;}
  int16_t getUSRangeMM(){return (_USSensRange*10);}
  int16_t getLSRRangeL(){return _LSRRangeL;}
  int16_t getLSRRangeR(){return _LSRRangeR;}
  int16_t getLSRRangeAlt(){return _LSRRangeA;}
  int16_t getLSRRangeB(){return _LSRRangeA;}  
  int16_t getLSRRangeU(){return _LSRRangeU;}
  int16_t getLSRRangeD(){return _LSRRangeD;}
  lastCollision_t* getLastCollision(){return &_lastCol;}

  void resetFlags(){
    _collisionFlag = false;
    _collisionUSFlag = false;
    _collisionBumperFlag = false;
    _collisionNervSys = B00000000;
    _collisionLSRFlagL = false;
    _collisionLSRFlagR = false;
    _collisionLSRFlagU = false;
    _collisionLSRFlagD = false;
  }

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
  void _updateColLSRs();
  void _updateAltLSR();
  void _updateUpDownLSRs();

  //---------------------------------------------------------------------------
  // Check all ranges and escape decision tree
  //---------------------------------------------------------------------------
  bool _updateCheckVec();
  void _updateEscapeDecision(); // Escape decision tree

  //---------------------------------------------------------------------------
  // LASER INIT: set all I2C addresses
  //---------------------------------------------------------------------------
  void _initLSR(byte sendByte, Adafruit_VL53L0X* LSRObj, bool* LSROn,
                uint8_t LSRAddr,char LSRStr);
  void _setLSRAddrs();

  //---------------------------------------------------------------------------
  // Helper functions
  //---------------------------------------------------------------------------
  void _sendByteWithI2C(byte inByte);

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
  bool _collisionFlag = false;
  bool _collisionUSFlag = false;
  uint16_t _halfBodyLengMM = 80;

  byte _collisionNervSys = B00000000;
  bool _collisionBumperFlag = false;
  bool _collisionBeepBeepFlag = false;
  uint16_t _collisionCount = 0;

  CollisionEscaper _escaper;

  // Check flags for all collision sensors
  bool _checkAllFlag = false;
  uint8_t _checkNum = 7;
  uint8_t _checkVec[7] = {0,0,0,0,0,0,0}; //_checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
  uint16_t _checkAllInt = 100;
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
  int16_t _USUpdateTime = 137;  // ms, set to prime number (100+timeout)
  Timer _USTimer = Timer();

  // LASER ranger variables
  uint16_t _resetDelay = 100;
  int16_t _LSRColDistClose = _halfBodyLengMM;  // mm
  int16_t _LSRColDistFar = 120;   //2*_halfBodyLengMM;   // mm
  int16_t _LSRColDistSlowD = 240; //4*_halfBodyLengMM; // mm
  int16_t _LSRColDistLim = 40;    // mm  
  int16_t _LSRAltDist = 80;       // mm
  bool _collisionLSRFlagL = false;
  bool _collisionLSRFlagR = false;
  bool _collisionLSRFlagB = false;

  // LSR - UP - DONT CHANGE!!!
  int16_t _LSRUpDistFar = 220;    // mm
  int16_t _LSRUpDistClose = 180;   // mm
  int16_t _LSRUpDistLim = 40;     // mm  
  // LSR- DWN - DONT CHANGE!!!
  int16_t _LSRDownCliffDistFar = 170, _LSRDownColDistFar = 90;     // mm
  int16_t _LSRDownCliffDistClose = 160, _LSRDownColDistClose = 70;   // mm
  int16_t _LSRDownCliffDistLim = 2000, _LSRDownColDistLim = 20;       // mm
  int16_t _LSRDownDistCent = 120; // actually measured closer to 145mm
  bool _collisionLSRFlagU = false, _collisionLSRFlagD = false;

  // Actual range values in mm
  bool _LSRLOn = false, _LSRROn = false, _LSRAOn = false;
  int16_t _LSRRangeL=0, _LSRRangeR=0, _LSRRangeA=0; //mm
  bool _LSRFlagL=false, _LSRFlagR=false, _LSRFlagA=false;
  bool _LSRL_TO=false, _LSRR_TO=false, _LSRA_TO=false;

  bool _LSRUOn = false, _LSRDOn = false;
  int16_t _LSRRangeU=0, _LSRRangeD=0; //mm
  bool _LSRFlagU=false, _LSRFlagD=false;
  bool _LSRU_TO=false, _LSRD_TO=false;
  
  uint16_t _colLSRUpdateTime = 101;
  Timer _colLSRTimer = Timer();
  uint16_t _altLSRUpdateTime = 101;
  Timer _altLSRTimer = Timer();
  uint16_t _upDownLSRUpdateTime = 101;
  Timer _upDownLSRTimer = Timer();
  
  // Objects for the laser ranger vl53l0x
  Adafruit_VL53L0X _laserL = Adafruit_VL53L0X();
  Adafruit_VL53L0X _laserR = Adafruit_VL53L0X();
  Adafruit_VL53L0X _laserA = Adafruit_VL53L0X();
  Adafruit_VL53L0X _laserU = Adafruit_VL53L0X();
  Adafruit_VL53L0X _laserD = Adafruit_VL53L0X();

  // I2C send byte 
  byte _toSend = B00000000;

  // Data structure for info on last collision
  lastCollision_t _lastCol;
};
#endif // COLLISIONMANAGER_H
