//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS - COLLISION
//---------------------------------------------------------------------------
/*
The collision class is part of the PetBot (PB) program.

Author: Lloyd Fletcher
Date Created: 28th Aug. 2021
Date Edited:  28th Aug. 2021
*/
#ifndef COLLISION_H
#define COLLISION_H

#include <Arduino.h>
#include <Ultrasonic.h>
#include "Adafruit_VL53L0X.h"
#include "Mood.h"
#include "Move.h"
#include "Task.h"
#include "Timer.h"

// COLLISION
#define ADDR_NERVSYS 9 
#define COLL_USSENS 7

#define ESCAPE_NOREV 1
#define ESCAPE_REV 0

#define COLL_SLOWD 1
#define COLL_FAR 2
#define COLL_CLOSE 3

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

struct lastCollision_t{
  uint8_t checkVec[7] = {0,0,0,0,0,0,0}; 
  uint16_t USRange = 0;
  uint16_t LSRRangeL = 0,LSRRangeR = 0;
  uint16_t LSRRangeU = 0,LSRRangeD = 0;
  uint8_t escCount = 0; 
  float escDist = 0.0, escAng = 0.0;
};
  
class Collision{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  Collision(Mood* inMood, Task* inTask, Move* inMove, Ultrasonic* ultrasonicSens){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _ultrasonicSens = ultrasonicSens;
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    // Start the ultrasonic timer  
    _USTimer.start(0);
    _colLSRTimer.start(0);
    _altLSRTimer.start(0);
    _upDownLSRTimer.start(0);
    _BMPRTimer.start(0);
    _slowDownTimer.start(0);    
    _setLSRAddrs();
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    //uint32_t startTime = micros(); 

    // If a new task is generated turn back on collision detecttion
    if(_taskObj->getNewTaskFlag()){
      setEnabledFlag(true);      
    }
            
    // COLLISION SENSOR: Run ultrasonic ranging, based on update time interval
    if(_USTimer.finished()){  
      _USTimer.start(_USUpdateTime);
      checkUS();
    }
    
    // COLLISION SENSOR: Run laser ranging   
    checkAltLSR();      // Timing handled internally
    checkColLSRs();     // Timing handled internally
    checkUpDownLSRs();  // Timing handled internally

    // COLLISION SENSOR: Bumper Switches slaved to Nervous System
    if(_BMPRTimer.finished()){
      _BMPRTimer.start(_BMPRUpdateTime);
      checkNervSys();
    }

    // COLLISION ACTION: If 
    if(_checkAllTimer.finished()){
      _checkAllTimer.start(_checkAllInt);
      _updateCheckVec();

      if(_checkAllFlag && _slowDownTimer.finished()){
        _slowDownTimer.start(_slowDownInt);
        _moveObj->setSpeedByColFlag(true);
      }
      else if(!_checkAllFlag && _slowDownTimer.finished()){
        _moveObj->setSpeedByColFlag(false); 
      }
    }
    
    if(!_slowDownTimer.finished()){
      _moveObj->setSpeedByColFlag(true);
    }

    // DISABLED: If collision detection is turned off set flags to false and return
    // Doing this last allows ranges to update but resets flags
    if(!_isEnabled){resetFlags();}

    //uint32_t endTime = micros();
    //Serial.println(endTime-startTime);
  }

  //---------------------------------------------------------------------------
  // CHECK SENSOR FUNCTIONS
  //---------------------------------------------------------------------------
  void checkUS(){  
    _USSensRange = _ultrasonicSens->MeasureInCentimeters();
    if(_USSensRange <= _USColDistLim){_USSensRange = 400;}
    
    if(_USSensRange <= _USColDistClose){
      _collisionFlag = true;
      _collisionUSFlag = true;
    }
    else if(_USSensRange <= _USColDistFar){
      _collisionFlag = true;
      _collisionUSFlag = true;
    }
  }
  
  void checkNervSys(){
    // Request a byte worth of digital pins from the follower Xiao
    Wire.requestFrom(ADDR_NERVSYS,1);
    // Read a byte from the follower
    byte nervSysFlags = B00000000;
    while(Wire.available()){
      nervSysFlags = Wire.read();   
    }
    //Serial.println(nervSysFlags,BIN);
    _collisionNervSys = nervSysFlags;
  
    if((nervSysFlags & B00000001) == B00000001){
      //Serial.println("Front Left Bumper");
      _collisionFlag = true;
      _collisionBumperFlag = true;
    }
    if ((nervSysFlags & B00000010) == B00000010){
      //Serial.println("Front Right Bumper");
      _collisionFlag = true;
      _collisionBumperFlag = true;
    }

    // If the bumpers are hit too many times decrease mood
    if(_collisionBumperFlag){
      _BMPRCount++;
      if(_BMPRCount >= _BMPRThres){
        _moodObj->decMoodScore();
        _BMPRCount = 0;
      }
    }
  }

  void checkColLSRs(){
    if(!_LSRLOn || !_LSRROn){return;}
    
    //------------------------------------------------------
    // START RANGING
    if(_colLSRTimer.finished()){
      _colLSRTimer.start(_colLSRUpdateTime);
      _laserL.startRange();
      _laserR.startRange();
      _LSRFlagL = false;
      _LSRFlagR = false;
    }

    //------------------------------------------------------
    // CHECK LASER RANGES
    if(_laserL.isRangeComplete() && !_LSRFlagL){
      _LSRRangeL = _laserL.readRangeResult();
      if(_LSRRangeL <= _LSRColDistLim){_LSRRangeL = 9999;}
      
      _LSRL_TO = _laserL.timeoutOccurred();
      _LSRFlagL = true;

      if(!_LSRL_TO){
        if(_LSRRangeL <= _LSRColDistClose){
          _collisionLSRFlagL = true;
          _collisionFlag = true;
        }
        else if(_LSRRangeL <= _LSRColDistFar){
          _collisionLSRFlagL = true;
          _collisionFlag = true;  
        }
      }
    }
    if(_laserR.isRangeComplete() && !_LSRFlagR){
      _LSRRangeR = _laserR.readRangeResult();
      if(_LSRRangeR <= _LSRColDistLim){_LSRRangeR = 9999;}
      
      _LSRR_TO = _laserR.timeoutOccurred();
      _LSRFlagR = true;

      if(!_LSRR_TO){
        if(_LSRRangeR <= _LSRColDistClose){
          _collisionLSRFlagR = true;
          _collisionFlag = true;
        }
        else if(_LSRRangeR <= _LSRColDistFar){
          _collisionLSRFlagR = true;
          _collisionFlag = true;
        }
      }
    }
  }

  void checkAltLSR() {
    //------------------------------------------------------
    // START RANGING
    if(_altLSRTimer.finished()){
      _altLSRTimer.start(_altLSRUpdateTime);
      _laserA.startRange();
      _LSRFlagA = false;
    }

    //------------------------------------------------------
    // CHECK LASER RANGE
    if(_laserA.isRangeComplete() && !_LSRFlagA){
      _LSRRangeA = _laserA.readRangeResult();
      _LSRA_TO = _laserL.timeoutOccurred();
      _LSRFlagA = true;

      if((_LSRRangeA >= _LSRAltDist) && !_LSRA_TO){
        _collisionLSRFlagB = true;
      }
      else{
        _collisionLSRFlagB = false;
      }
    }
  }

  void checkUpDownLSRs(){
    if(!_LSRUOn || !_LSRDOn){return;}
    
    //------------------------------------------------------
    // START RANGING
    if(_upDownLSRTimer.finished()){
      _upDownLSRTimer.start(_upDownLSRUpdateTime);
      _laserU.startRange();
      _laserD.startRange();
      _LSRFlagU = false;
      _LSRFlagD = false;
    }

    //------------------------------------------------------
    // CHECK LASER RANGES
    if(_laserU.isRangeComplete() && !_LSRFlagU){
      _LSRRangeU = _laserU.readRangeResult();
      if(_LSRRangeU <= _LSRUpDistLim){_LSRRangeU = 9999;}
      
      _LSRU_TO = _laserU.timeoutOccurred();
      _LSRFlagU = true;

      if(!_LSRU_TO){
        if(_LSRRangeU <= _LSRUpDistClose){
          _collisionLSRFlagU = true;
          _collisionFlag = true;
        }
        else if(_LSRRangeU <= _LSRUpDistFar){
          _collisionLSRFlagU = true;
          _collisionFlag = true;  
        }
      }
    }
    
    if(_laserD.isRangeComplete() && !_LSRFlagD){
      _LSRRangeD = _laserD.readRangeResult();
      if(_LSRRangeD <= _LSRDownColDistLim || _LSRRangeD >= _LSRDownCliffDistLim){
        _LSRRangeD = _LSRDownDistCent;
      }
      
      _LSRD_TO = _laserD.timeoutOccurred();
      _LSRFlagD = true;

      if(!_LSRD_TO){
        if((_LSRRangeD <= _LSRDownColDistClose)||(_LSRRangeD >= _LSRDownCliffDistClose)){
          _collisionLSRFlagD = true;
          _collisionFlag = true; 
        }
        else if((_LSRRangeD <= _LSRDownColDistFar)||(_LSRRangeD >= _LSRDownCliffDistFar)){
          _collisionLSRFlagD = true;
          _collisionFlag = true;
        }
      }
    }
  }

  //---------------------------------------------------------------------------
  // GET, SET AND RESET 
  //---------------------------------------------------------------------------
  bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

  uint16_t getCount(){return _collisionCount;}
  void incCount(){_collisionCount++;}
  void resetCount(){_collisionCount = 0;}
  
  bool getBeepBeepFlag(){return _collisionBeepBeepFlag;}
  void setBeepBeepFlag(bool inFlag){_collisionBeepBeepFlag = inFlag;}

  // GET COLLISION FLAGS
  bool getDetectFlag(){return _collisionFlag;} 
  bool getBumperFlag(){return _collisionBumperFlag;}

  bool getColLSRFlagL(){return _collisionLSRFlagL;}
  bool getColLSRFlagR(){return _collisionLSRFlagR;}
  bool getColLSRFlagB(){return _collisionLSRFlagB;}
  bool getColLSRFlagU(){return _collisionLSRFlagU;}
  bool getColLSRFlagD(){return _collisionLSRFlagD;}
  bool getAltFlag(){return _collisionLSRFlagB;}
  bool getColUSFlag(){return _collisionUSFlag;}
    
  // GET RANGES
  int16_t getUSRange(){return _USSensRange;}
  int16_t getUSRangeMM(){return (_USSensRange*10);}
  int16_t getLSRRangeL(){return _LSRRangeL;}
  int16_t getLSRRangeR(){return _LSRRangeR;}
  int16_t getLSRRangeAlt(){return _LSRRangeA;}
  int16_t getLSRRangeB(){return _LSRRangeA;}  
  int16_t getLSRRangeU(){return _LSRRangeU;}
  int16_t getLSRRangeD(){return _LSRRangeD;}
  
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

  //============================================================================
  // COLLISION ESCAPE FUNCTIONS - USES MOVE OBJ
  //============================================================================ 
  void setEscapeStart(){   
    _updateCheckVec();    // Check all collision sensors - used for decision tree
    _updateEscapeVars();  // This is the decision tree
  }

  void escape(){
    if(_escapeCount == 0){ // Use the first escape count to reverse by a set distance
      int8_t isComplete = _moveObj->toDistCtrlSpd(_escapeDist);
      if(isComplete > 0){ // this portion of the escape is complete
        _escapeCount = 1;   
      }
    }
    else if(_escapeCount == 1){ 
      int8_t isComplete = _moveObj->turnToAngleCtrlSpd(_escapeAngle);
      if(isComplete > 0){ // this portion of the escape is complete
        _escapeCount = 2; 
      }
    }
    else{
      _moveObj->stop();
    }  
  }
  
  bool getEscapeFlag(){
    bool outFlag = false;
    if(_escapeCount <= 1){
      outFlag = true;
    }
    return outFlag;  
  }

  int8_t getEscapeTurn(){
    _updateCheckVec();    // Check all collision sensors - used for decision tree
    _updateEscapeVars();  // This is the decision tree
    if(_escapeAngle > 0.0){
      return MOVE_B_LEFT;
    }
    else{
      return MOVE_B_RIGHT;
    }
  }

  lastCollision_t* getLastCollision(){
    return &_lastCol;
  }

private:
  //---------------------------------------------------------------------------
  // CHECK ALL RANGES AND COLLISION DECISION TREE
  //---------------------------------------------------------------------------
  bool _updateCheckVec(){
    // uint8_t _checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
    // NOTE: if's have to have most severe case first!

    // Bumpers
    if(((_collisionNervSys & B00000001) == B00000001)){_checkVec[0] = COLL_CLOSE;}
    else{_checkVec[0] = 0;}
    if(((_collisionNervSys & B00000010) == B00000010)){_checkVec[1] = COLL_CLOSE;}
    else{_checkVec[1] = 0;}

    // Ultrasonic Ranger
    if(_USSensRange <= _USColDistClose){_checkVec[2] = COLL_CLOSE;}
    else if(_USSensRange <= _USColDistFar){_checkVec[2] = COLL_FAR;}
    else if(_USSensRange <= _USColDistSlowD){_checkVec[2] = COLL_SLOWD;}
    else{_checkVec[2] = 0;}
    
    // Laser - Left
    if(_LSRRangeL <= _LSRColDistClose){_checkVec[3] = COLL_CLOSE;}
    else if(_LSRRangeL <= _LSRColDistFar){_checkVec[3] = COLL_FAR;}
    else if(_LSRRangeL <= _LSRColDistSlowD){_checkVec[3] = COLL_SLOWD;} 
    else{_checkVec[3] = 0;}

    // Laser - Right
    if(_LSRRangeR <= _LSRColDistClose){_checkVec[4] = COLL_CLOSE;}
    else if(_LSRRangeR <= _LSRColDistFar){_checkVec[4] = COLL_FAR;}
    else if(_LSRRangeR <= _LSRColDistSlowD){_checkVec[4] = COLL_SLOWD;}
    else{_checkVec[4] = 0;}

    // Laser - Up Angle
    if(_LSRRangeU <= _LSRUpDistClose){_checkVec[5] = COLL_CLOSE;}
    else if(_LSRRangeU <= _LSRUpDistFar){_checkVec[5] = COLL_FAR;}
    else{_checkVec[5] = 0;}

    // Laser - Down Angle - TODO, fix this so we know which is which
    if(_LSRRangeD >= _LSRDownCliffDistClose){_checkVec[6] = COLL_CLOSE;}
    else if(_LSRRangeD <= _LSRDownColDistClose){_checkVec[6] = COLL_CLOSE;}
    else if(_LSRRangeD <= _LSRDownColDistFar){_checkVec[6] = COLL_FAR;}
    else if(_LSRRangeD >= _LSRDownCliffDistFar){_checkVec[6] = COLL_FAR;}
    else{_checkVec[6] = 0;}

    // If anything is tripped set this flag true
    _checkAllFlag = false; 
    for(uint8_t ii=0;ii<_checkNum;ii++){
      if(_checkVec[ii] > 0){
        _checkAllFlag = true;
        break;
      }
    }
    return _checkAllFlag;
  }

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  // ESCAPE DECISION TREE
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  void _updateEscapeVars(){
    // _checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
    // NOTE: first thing in the tree is dealt with first

    // 1) BMPL: reverse a full body length and turn +90 deg away
    if(_checkVec[0] == COLL_CLOSE){
      _escapeCount = ESCAPE_REV;
      _escapeDist = 1.0*_defRevDist;
      _escapeAngle = -1.0*_defHardTurn;
    }
    // 2) BMPR: reverse a full body length and turn -90 deg away
    else if(_checkVec[1] == COLL_CLOSE){
      _escapeCount = ESCAPE_REV;
      _escapeDist = 1.0*_defRevDist;
      _escapeAngle = 1.0*_defHardTurn;
    }
    // 7) LSRD: check cliff-close/far, check obstacle-close/far 
    else if(_checkVec[6] >= COLL_FAR){
      if(_checkVec[6] == COLL_CLOSE){
        _escapeCount = ESCAPE_REV;
        _escapeDist = 1.0*_defRevDist;
        _escapeAngle = _getRandTurnDir()*_defHardTurn;
      }
      else{
        _escapeCount = ESCAPE_NOREV;
        _escapeDist = 0.0;
        _escapeAngle = _getRandTurnDir()*_defHardTurn;        
      }    
    }
    // 4) LSRL: check far (norev,+45deg), check close (rev,+90deg)
    else if(_checkVec[3] >= COLL_FAR){
      if(_checkVec[3] == COLL_CLOSE){
        _escapeCount = ESCAPE_REV;
        _escapeDist = 0.8*_defRevDist;
        _escapeAngle = -1.25*_defModTurn;
      }
      else{
        _escapeCount = ESCAPE_NOREV;
        _escapeDist = 0.0;
        _escapeAngle = -1.0*_defModTurn;        
      }

      if(_checkVec[4] >= COLL_FAR){ // If the right laser is tripped as well then we are in a corner, do a 180
        _escapeAngle = -180.0 + _getRandTurnDir()*float(random(0,45));         
      }
      else if(_checkVec[2] >= COLL_FAR){ // If the ultrasonic sensor is tripped as well then turn harder
        _escapeAngle = -1.0*_defHardTurn;
      }
    }
    // 5) LSRR: check far (norev,-45deg), check close (rev,-90deg)
    else if(_checkVec[4] >= COLL_FAR){
      if(_checkVec[4] == COLL_CLOSE){
        _escapeCount = ESCAPE_REV;
        _escapeDist = 0.8*_defRevDist;
        _escapeAngle = 1.25*_defModTurn;
      }
      else{
        _escapeCount = ESCAPE_NOREV;
        _escapeDist = 0.0;
        _escapeAngle = 1.0*_defModTurn;        
      }  

      if(_checkVec[3] >= COLL_FAR){ // If the right laser is tripped as well then we are in a corner, do a 180
        _escapeAngle = 180.0 + _getRandTurnDir()*float(random(0,45));         
      }
      else if(_checkVec[2] >= COLL_FAR){ // If the ultrasonic sensor is tripped as well then turn harder
        _escapeAngle = -1.0*_defHardTurn;
      }
    }
    // 3) US: check far (norev,+/-45deg), check close (rev,+/-90deg)
    else if(_checkVec[2] >= COLL_FAR){
      if(_checkVec[2] == COLL_CLOSE){
        _escapeCount = ESCAPE_REV;
        _escapeDist = _defRevDist;
        _escapeAngle = _getRandTurnDir()*_defHardTurn;
      }
      else{
        _escapeCount = ESCAPE_NOREV;
        _escapeDist = 0.0;
        _escapeAngle = _getRandTurnDir()*_defHardTurn;        
      }
    }
    // 6) LSRU: check overhang-close/far 
    else if(_checkVec[5] >= COLL_FAR){
      if(_checkVec[5] == COLL_CLOSE){
        _escapeCount = ESCAPE_REV;
        _escapeDist = _defRevDist;
        _escapeAngle = _getRandTurnDir()*_defHardTurn;
      }
      else{
        _escapeCount = ESCAPE_NOREV;
        _escapeDist = 0.0;
        _escapeAngle = _getRandTurnDir()*_defHardTurn;        
      }         
    }

    // Update the last collision variable after the decision tree
    for(uint8_t ii=0;ii<_checkNum;ii++){
      _lastCol.checkVec[ii] = _checkVec[ii];
    }
    _lastCol.USRange = _USSensRange*10;
    _lastCol.LSRRangeL = _LSRRangeL;
    _lastCol.LSRRangeR = _LSRRangeR;
    _lastCol.LSRRangeU = _LSRRangeU;
    _lastCol.LSRRangeD = _LSRRangeD;
    _lastCol.escCount = _escapeCount;
    _lastCol.escDist = _escapeDist;
    _lastCol.escAng = _escapeAngle;

    // Plot debug information
    #if defined(COLL_DEBUG_DECISIONTREE)
    Serial.println();
    Serial.println("CheckVec=[BL,BR,US,LL,LR,LU,LD,]");
    Serial.print("CheckVec=[");
    for(uint8_t ii=0;ii<_checkNum;ii++){
      Serial.print(" ");Serial.print(_checkVec[ii]);Serial.print(","); 
    }
    Serial.println("]");
    Serial.print("US="); Serial.print(_USSensRange*10); Serial.print("mm, ");
    Serial.print("LL="); Serial.print(_LSRRangeL); Serial.print("mm, ");
    Serial.print("LR="); Serial.print(_LSRRangeR); Serial.print("mm");
    Serial.println();
    Serial.print("LU="); Serial.print(_LSRRangeU); Serial.print("mm, ");
    Serial.print("LD="); Serial.print(_LSRRangeD); Serial.print("mm");
    Serial.println();
    Serial.print("Esc,Count="); Serial.print(_escapeCount);
    Serial.print(", Dist="); Serial.print(_escapeDist);
    Serial.print(", Angle="); Serial.print(_escapeAngle);
    Serial.println();
    #endif 
  }

  //---------------------------------------------------------------------------
  // LASER INIT - set all I2C addresses
  //---------------------------------------------------------------------------
  void _initLSR(byte sendByte, Adafruit_VL53L0X* LSRObj, bool* LSROn,
                uint8_t LSRAddr,char LSRStr){
    _sendByteWithI2C(sendByte);
    delay(_resetDelay);
    if(!LSRObj->begin(LSRAddr)){
      Serial.print(F("COLLISION: FAILED to init laser "));
      Serial.println(LSRStr);
      *LSROn = false;
    }
    else{
      Serial.print(F("COLLISION: initialised laser "));
      Serial.println(LSRStr);
      *LSROn = true;
    }
    delay(_resetDelay);
  }
  
  void _setLSRAddrs(){
    //-----------------------------------------
    // Reset all laser sensors - set all low
    _toSend = B00000000;
    _sendByteWithI2C(_toSend);
    delay(_resetDelay);
    
    // Turn on all sensors - set all high
    _toSend = B00111110;
    _sendByteWithI2C(_toSend);
    delay(_resetDelay);
    char LSRStr = 'L'; 

    //-----------------------------------------
    // Activate first laser sensor
    _toSend = B00000010;
    LSRStr = 'L'; 
    _initLSR(_toSend,&_laserL,&_LSRLOn,ADDR_LSR_L,LSRStr);

    //-----------------------------------------
    // Activate second laser sensor
    _toSend = B00000110;
    LSRStr = 'R'; 
    _initLSR(_toSend,&_laserR,&_LSRROn,ADDR_LSR_R,LSRStr);

    //-----------------------------------------  
    // Activate third laser sensor
    _toSend = B00001110;
    LSRStr = 'A'; 
    _initLSR(_toSend,&_laserA,&_LSRAOn,ADDR_LSR_A,LSRStr);

    //----------------------------------------- 
    // Activate fourth laser sensor
    _toSend = B00011110;
    LSRStr = 'U'; 
    _initLSR(_toSend,&_laserU,&_LSRUOn,ADDR_LSR_U,LSRStr);

    //----------------------------------------- 
    // Activate fifth laser sensor
    _toSend = B00111110;
    LSRStr = 'D'; 
    _initLSR(_toSend,&_laserD,&_LSRDOn,ADDR_LSR_D,LSRStr);
  }
  //---------------------------------------------------------------------------
  // PRIVATE HELPER FUNCTIONS
  //---------------------------------------------------------------------------
  void _sendByteWithI2C(byte inByte){
    Wire.beginTransmission(ADDR_FOLLBOARD);
    Wire.write(inByte);
    Wire.endTransmission();
  }

  float _getRandTurnDir(){
    uint8_t randVal = random(0,2);
    float sendVal = 1.0;
    if(randVal == 1){
      sendVal = 1.0;
    }
    else{
      sendVal = -1.0;
    }
    return sendVal;
  }

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  Mood* _moodObj;
  Task* _taskObj;
  Move* _moveObj;
  
  // ULTRASONIC SENSOR
  Ultrasonic* _ultrasonicSens;
  
  // COLLISION OBJ - variables
  bool _isEnabled = true;
  bool _collisionFlag = false;
  bool _collisionUSFlag = false;
  uint16_t _halfBodyLengMM = 90;

  byte _collisionNervSys = B00000000;
  bool _collisionBumperFlag = false;
  bool _collisionBeepBeepFlag = false;
  uint16_t _collisionCount = 0;

  // COLLISION OBJ - Check flags for all collision sensors
  bool _checkAllFlag = false;
  uint8_t _checkNum = 7;
  uint8_t _checkVec[7] = {0,0,0,0,0,0,0}; //_checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
  uint16_t _checkAllInt = 100;
  Timer _checkAllTimer = Timer();

  // COLLISION OBJ - time to slow down if sensor tripped
  uint16_t _slowDownInt = 500;
  Timer _slowDownTimer = Timer();

  // COLLISION OBJ - Bumper Variables
  int8_t _BMPRCount = 0, _BMPRThres = 13;
  int16_t _BMPRUpdateTime = 101;
  Timer _BMPRTimer = Timer();
  
  // COLLISION OBJ - Ultrasonic Sensor Variables
  int16_t _USSensRange = 1000;
  int16_t _USColDistClose = _halfBodyLengMM/10; // cm
  int16_t _USColDistFar = 2*_halfBodyLengMM/10;  // cm  
  int16_t _USColDistSlowD = 4*_halfBodyLengMM/10; // cm
  int16_t _USColDistLim = 4;    // cm 
  int16_t _USUpdateTime = 137;  // ms, set to prime number (100+timeout)
  Timer _USTimer = Timer();

  // COLLISION OBJ - LASER SENSOR VARIABLES
  uint16_t _resetDelay = 100;
  int16_t _LSRColDistClose = _halfBodyLengMM;  // mm
  int16_t _LSRColDistFar = 2*_halfBodyLengMM;   // mm
  int16_t _LSRColDistSlowD = 4*_halfBodyLengMM; // mm
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
  
  bool _collisionLSRFlagU = false;
  bool _collisionLSRFlagD = false;

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
  
  // Objects for the vl53l0x
  Adafruit_VL53L0X _laserL = Adafruit_VL53L0X();
  Adafruit_VL53L0X _laserR = Adafruit_VL53L0X();
  Adafruit_VL53L0X _laserA = Adafruit_VL53L0X();
  Adafruit_VL53L0X _laserU = Adafruit_VL53L0X();
  Adafruit_VL53L0X _laserD = Adafruit_VL53L0X();

  // I2C send byte 
  byte _toSend = B00000000;

  // COLLISION/MOVE - Escape Variables
  uint8_t _escapeCount = 3;
  float _escapeAngle = 45.0;
  float _escapeDist = 180.0;
  const static uint8_t _escapeNumSteps = 3;
  float _defModTurn = 45.0, _defHardTurn = 90.0;
  float _defRevDist = -180.0;

  // COLLISION - Last Collision Data
  lastCollision_t _lastCol;
};
#endif // COLLISION_H
