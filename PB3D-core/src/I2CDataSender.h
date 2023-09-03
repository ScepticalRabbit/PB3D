//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS: I2CDataSender
//-----------------------------------------------------------------------------
/*
TODO

Author: Lloyd Fletcher
Date Created: 11th Dec. 2022
Date Edited:  11th Dec. 2022
*/

#ifndef I2CDATASENDER_H
#define I2CDATASENDER_H

#include <Wire.h>
#include "CollisionManager.h"
#include "Timer.h"
#include "IMU.h"
#include "Navigation.h"
#include "StateData.h"

// Address for nervous system peripherial Xiao
#define NERVSYS_ADDR 9

// Debug flags
//#define I2CDATASENDER_DEBUG_PRINT

//----------------------------------------------------------------------------
// CLASS
class I2CDataSender{
public:
  I2CDataSender(CollisionManager* inCollision, Mood* inMood, Task* inTask, Move* inMove,
                IMU* inIMU, Navigation* inNav){
    _collisionObj = inCollision;
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _IMUObj = inIMU;
    _navObj = inNav;
  }

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin(){
    _initStateData(&_currState);
    _updateStateData(&_currState);
    _I2CTimer.start(_I2CTime);
    
    #if defined(I2CDATASENDER_DEBUG_PRINT)
      Serial.println(F("INITIAL DATA STRUCT"));
      _printStateData(&_currState);
    #endif

    _lastCol = _collisionObj->getLastCollision();
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during LOOP
  void update(){
    if(!_isEnabled){return;}

    if(_I2CTimer.finished()){
      _I2CTimer.start(_I2CTime);
      _sendTimer.start(0);

      _updateStateData(&_currState);
  
      Wire.beginTransmission(NERVSYS_ADDR);
      Wire.write(_currState.dataPacket,PACKET_SIZE);
      Wire.endTransmission();

      #if defined(I2CDATASENDER_DEBUG_PRINT)
        Serial.print(F("I2C Send Time: "));    
        Serial.print(_sendTimer.getTime());
        Serial.println(F("ms"));
        
        Serial.println(F("SENT DATA STRUCTURE:"));
        _printStateData(&_currState);
      #endif
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS

  // UPDATE: only used by this class so not in the header
  void _updateStateData(dataPacket_t* inState){
  #if defined(STATEDATA_LASTCOL)
    // TIME
    inState->state.onTime = millis();
    // LAST COLLISION
    for(uint8_t ii=0;ii<7;ii++){
        inState->state.checkVec[ii] = _lastCol->checkVec[ii];
    }
    inState->state.USRange = _lastCol->USRange;
    inState->state.LSRRangeL = _lastCol->LSRRangeL;
    inState->state.LSRRangeR = _lastCol->LSRRangeR;
    inState->state.LSRRangeU = _lastCol->LSRRangeU;
    inState->state.LSRRangeD = _lastCol->LSRRangeD;
    inState->state.escCount = _lastCol->escCount; 
    inState->state.escDist = _lastCol->escDist; 
    inState->state.escAng = _lastCol->escAng;
  #elif defined(STATEDATA_NAV)
    // TIME
    inState->state.onTime = millis();
    // MOVE
    inState->state.wheelSpeedL = _moveObj->getEncSpeedL();
    inState->state.wheelSpeedR = _moveObj->getEncSpeedR();
    // IMU
    inState->state.IMUHead = _IMUObj->getHeadAng();
    inState->state.IMUPitch = _IMUObj->getPitchAng();
    inState->state.IMURoll = _IMUObj->getRollAng();
    // Navigation
    inState->state.navPosX = _navObj->getPosX();
    inState->state.navPosY = _navObj->getPosY();
    inState->state.navVelX = _navObj->getVelX();
    inState->state.navVelY = _navObj->getVelY();
    inState->state.navVelC = _navObj->getVelC();
    inState->state.navHead = _navObj->getHeading();  
  #else
    // TIME
    inState->state.onTime = millis();
    // MOOD
    inState->state.mood = _moodObj->getMood();
    inState->state.moodScore = _moodObj->getMoodScore();
    // TASK  
    inState->state.task = _taskObj->getTask();
    // MOVE
    inState->state.moveBasic = _moveObj->getBasicMove();
    inState->state.moveCompound = _moveObj->getCompoundMove();
    inState->state.escapeFlag = _moveObj->getEscapeFlag();
    inState->state.setForwardSpeed = _moveObj->getForwardSpeed();
    inState->state.wheelSpeedL = _moveObj->getEncSpeedL();
    inState->state.wheelSpeedR = _moveObj->getEncSpeedL();
    inState->state.wheelECountL = _moveObj->getEncCountL();
    inState->state.wheelECountR = _moveObj->getEncCountR();
    // COLLISON - Latches
    inState->state.colFlag = _collisionObj->getDetectFlag();
    inState->state.colBMPRs = _collisionObj->getBumperFlag();
    inState->state.colUSR = _collisionObj->getColUSFlag();
    inState->state.colLSRL = _collisionObj->getColLSRFlagL();
    inState->state.colLSRR = _collisionObj->getColLSRFlagR();
    inState->state.colLSRB = _collisionObj->getColLSRFlagB();
    inState->state.colLSRU = _collisionObj->getColLSRFlagU();
    inState->state.colLSRD = _collisionObj->getColLSRFlagD();
    // COLLISION - Ranges
    inState->state.colUSRRng = _collisionObj->getUSRangeMM();
    inState->state.colLSRLRng = _collisionObj->getLSRRangeL();
    inState->state.colLSRRRng = _collisionObj->getLSRRangeR();
    inState->state.colLSRBRng = _collisionObj->getLSRRangeB();
    inState->state.colLSRURng = _collisionObj->getLSRRangeU();
    inState->state.colLSRDRng = _collisionObj->getLSRRangeD();
  #endif
  }

  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  CollisionManager* _collisionObj;
  Mood* _moodObj;
  Task* _taskObj;
  Move* _moveObj;
  IMU* _IMUObj;
  Navigation* _navObj;
  lastCollision_t* _lastCol;
  
  //---------------------------------------------------------------------------
  // CLASS VARIABLES 
  bool _isEnabled = true;
  bool _startFlag = true;

  // DATA PACKET
  dataPacket_t _currState;
  bool _dataSwitch = false;

  // I2C VARIABLES
  uint16_t _I2CTime = STATEDATA_UPD_TIME;
  Timer _I2CTimer = Timer();
  Timer _sendTimer = Timer();
};
#endif 
