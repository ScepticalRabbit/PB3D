//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef I2CDATASENDER_H
#define I2CDATASENDER_H

#include <Arduino.h>
#include <Wire.h>
#include "CollisionManager.h"
#include "Timer.h"
#include "IMUSensor.h"
#include "Navigation.h"
#include "StateData.h"
#include "PB3DI2CAddresses.h"

// Debug flags
//#define I2CDATASENDER_DEBUG_PRINT

//----------------------------------------------------------------------------
// CLASS: I2CDataSender
class I2CDataSender{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR: pass in pointers to main objects and other sensors
  I2CDataSender(CollisionManager* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove,
              IMUSensor* inIMU, Navigation* inNav){
      _collisionObj = inCollision;
      _mood_manager = inMood;
      _task_manager = inTask;
      _move_manager = inMove;
      _IMUObj = inIMU;
      _navObj = inNav;
  }


  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin(){
    _initStateData(&_currState);
    _updateStateData(&_currState);
    _I2CTimer.start(_I2CTime);

    #if defined(I2CDATASENDER_DEBUG_PRINT)
        Serial.println(F("INITIAL DATA STRUCT"));
        _printStateData(&_currState);
    #endif

    _last_col = _collisionObj->get_last_collision();
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update(){
    if(!_is_enabled){return;}

    if(_I2CTimer.finished()){
        _I2CTimer.start(_I2CTime);
        _sendTimer.start(0);

        _updateStateData(&_currState);

        Wire.beginTransmission(ADDR_FOLLOW_XIAO_1);
        Wire.write(_currState.dataPacket,PACKET_SIZE);
        Wire.endTransmission();

        #if defined(I2CDATASENDER_DEBUG_PRINT)
        Serial.print(F("I2C Send Time: "));
        Serial.print(_sendTimer.get_time());
        Serial.println(F("ms"));

        Serial.println(F("SENT DATA STRUCTURE:"));
        _printStateData(&_currState);
        #endif
    }
  }

  //---------------------------------------------------------------------------
  // GET FUNCTIONS
  bool get_enabled_flag(){return _is_enabled;}

  //---------------------------------------------------------------------------
  // SET FUNCTIONS
  void set_enabled_flag(bool inFlag){_is_enabled = inFlag;}

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
            inState->state.checkVec[ii] = _last_col->check_vec[ii];
        }
        inState->state.ultrasonic_range = _last_col->ultrasonic_range;
        inState->state.LSRRangeL = _last_col->LSRRangeL;
        inState->state.LSRRangeR = _last_col->LSRRangeR;
        inState->state.LSRRangeU = _last_col->LSRRangeU;
        inState->state.LSRRangeD = _last_col->LSRRangeD;
        inState->state.escape_count = _last_col->escape_count;
        inState->state.escape_dist = _last_col->escape_dist;
        inState->state.escape_angle = _last_col->escape_angle;
    #elif defined(STATEDATA_NAV)
        // TIME
        inState->state.onTime = millis();
        // MOVE
        inState->state.wheelSpeedL = _move_manager->get_encoder_speed_left();
        inState->state.wheelSpeedR = _move_manager->get_encoder_speed_right();
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
        inState->state.mood = _mood_manager->getMood();
        inState->state.moodScore = _mood_manager->getMoodScore();
        // TASK
        inState->state.task = _task_manager->getTask();
        // MOVE
        inState->state.moveBasic = _move_manager->get_basic_move();
        inState->state.moveCompound = _move_manager->get_compound_move();
        inState->state.escapeFlag = _move_manager->get_escape_flag();
        inState->state.set_forward_speed = _move_manager->get_forward_speed();
        inState->state.wheelSpeedL = _move_manager->get_encoder_speed_left();
        inState->state.wheelSpeedR = _move_manager->get_encoder_speed_left();
        inState->state.wheelECountL = _move_manager->get_encoder_count_left();
        inState->state.wheelECountR = _move_manager->get_encoder_count_right();
        // COLLISON - Latches
        inState->state.colFlag = _collisionObj->get_detected();
        inState->state.colBMPRs = _collisionObj->get_bumper_flag();
        inState->state.colUSR = _collisionObj->getColUSFlag();
        inState->state.colLSRL = _collisionObj->getColLSRFlagL();
        inState->state.colLSRR = _collisionObj->getColLSRFlagR();
        inState->state.colLSRB = _collisionObj->getColLSRFlagB();
        inState->state.colLSRU = _collisionObj->getColLSRFlagU();
        inState->state.colLSRD = _collisionObj->getColLSRFlagD();
        // COLLISION - Ranges
        inState->state.colUSRRng = _collisionObj->get_ultrasonic_range_mm();
        inState->state.colLSRLRng = _collisionObj->getLSRRangeL();
        inState->state.colLSRRRng = _collisionObj->getLSRRangeR();
        inState->state.colLSRBRng = _collisionObj->getLSRRangeB();
        inState->state.colLSRURng = _collisionObj->getLSRRangeU();
        inState->state.colLSRDRng = _collisionObj->getLSRRangeD();
    #endif
  }

  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  CollisionManager* _collisionObj = NULL;
  MoodManager* _mood_manager = NULL;
  TaskManager* _task_manager = NULL;
  MoveManager* _move_manager = NULL;
  IMUSensor* _IMUObj = NULL;
  Navigation* _navObj = NULL;
  SLastCollision* _last_col = NULL;

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  bool _is_enabled = true;
  bool _start_flag = true;

  // DATA PACKET
  dataPacket_t _currState;
  bool _dataSwitch = false;

  // I2C VARIABLES
  uint16_t _I2CTime = STATEDATA_UPD_TIME;
  Timer _I2CTimer = Timer();
  Timer _sendTimer = Timer();
};
#endif
