//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: CollisionManager
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include "CollisionManager.h"

//-----------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//-----------------------------------------------------------------------------
CollisionManager::CollisionManager(Mood* inMood, Task* inTask, Move* inMove, Ultrasonic* ultrasonicSens){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _ultrasonicSens = ultrasonicSens;

    _escaper.setMoveObj(inMove);
}

//-----------------------------------------------------------------------------
// BEGIN: called during setup function before main loop
//-----------------------------------------------------------------------------
void CollisionManager::begin(){
    // Start the ultrasonic timer  
    _USTimer.start(0);
    _colLSRTimer.start(0);
    _altLSRTimer.start(0);
    _upDownLSRTimer.start(0);
    _BMPRTimer.start(0);
    _slowDownTimer.start(0);    
    _setLSRAddrs();
}

//-----------------------------------------------------------------------------
// UPDATE: called during every iteration of the main loop
//-----------------------------------------------------------------------------
void CollisionManager::update(){
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

    // COLLISION ACTION
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

//-----------------------------------------------------------------------------
void CollisionManager::checkUS(){  
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

//-----------------------------------------------------------------------------
void CollisionManager::checkNervSys(){
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

//-----------------------------------------------------------------------------
void CollisionManager::checkColLSRs(){
    if(!_LSRLOn || !_LSRROn){return;}

    // Start ranging
    if(_colLSRTimer.finished()){
        _colLSRTimer.start(_colLSRUpdateTime);
        _laserL.startRange();
        _laserR.startRange();
        _LSRFlagL = false;
        _LSRFlagR = false;
    }

    // Check if ranging complete and if so read the range
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

//-----------------------------------------------------------------------------
void CollisionManager::checkAltLSR() {
    // Start ranging
    if(_altLSRTimer.finished()){
        _altLSRTimer.start(_altLSRUpdateTime);
        _laserA.startRange();
        _LSRFlagA = false;
    }

    // Check if ranging complete and if so read the range
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

//-----------------------------------------------------------------------------
void CollisionManager::checkUpDownLSRs(){
    if(!_LSRUOn || !_LSRDOn){return;}

    // Start ranging
    if(_upDownLSRTimer.finished()){
        _upDownLSRTimer.start(_upDownLSRUpdateTime);
        _laserU.startRange();
        _laserD.startRange();
        _LSRFlagU = false;
        _LSRFlagD = false;
    }

    // Read laser ranges
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

//-----------------------------------------------------------------------------
void CollisionManager::setEscapeStart(){   
    _updateCheckVec();    // Check all collision sensors - used for decision tree
    _escaper.updateEscapeDecision(_checkVec);  // This is the decision tree
}

//-----------------------------------------------------------------------------
void CollisionManager::escape(){
    _escaper.escape();
}

//-----------------------------------------------------------------------------
bool CollisionManager::getEscapeFlag(){
    return _escaper.getEscapeFlag();
}

//-----------------------------------------------------------------------------
int8_t CollisionManager::getEscapeTurn(){
    _updateCheckVec();    // Check all collision sensors - used for decision tree
    return _escaper.getEscapeTurn(_checkVec);
}

//---------------------------------------------------------------------------
bool CollisionManager::_updateCheckVec(){
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

//-----------------------------------------------------------------------------
// ESCAPE DECISION TREE
void CollisionManager::_updateEscapeDecision(){
    // Forward to escaper
    _escaper.updateEscapeDecision(_checkVec);

    // Update the last collision variable after the decision tree
    for(uint8_t ii=0;ii<_checkNum;ii++){
        _lastCol.checkVec[ii] = _checkVec[ii];
    }
    _lastCol.USRange = _USSensRange*10;
    _lastCol.LSRRangeL = _LSRRangeL;
    _lastCol.LSRRangeR = _LSRRangeR;
    _lastCol.LSRRangeU = _LSRRangeU;
    _lastCol.LSRRangeD = _LSRRangeD;
    _lastCol.escCount = _escaper.getEscapeCount();
    _lastCol.escDist = _escaper.getEscapeDist();
    _lastCol.escAng = _escaper.getEscapeAngle();

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
    Serial.print("Esc,Count="); Serial.print(_lastCol.escCount);
    Serial.print(", Dist="); Serial.print(_lastCol.escDist);
    Serial.print(", Angle="); Serial.print(_lastCol.escAng);
    Serial.println();
    #endif 
}

//-----------------------------------------------------------------------------
void CollisionManager::_initLSR(byte sendByte, Adafruit_VL53L0X* LSRObj, bool* LSROn,
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

//-----------------------------------------------------------------------------
void CollisionManager::_setLSRAddrs(){
    // Reset all laser sensors - set all low
    _toSend = B00000000;
    _sendByteWithI2C(_toSend);
    delay(_resetDelay);

    // Turn on all sensors - set all high
    _toSend = B00111110;
    _sendByteWithI2C(_toSend);
    delay(_resetDelay);
    char LSRStr = 'L'; 

    // Activate first laser sensor
    _toSend = B00000010;
    LSRStr = 'L'; 
    _initLSR(_toSend,&_laserL,&_LSRLOn,ADDR_LSR_L,LSRStr);

    // Activate second laser sensor
    _toSend = B00000110;
    LSRStr = 'R'; 
    _initLSR(_toSend,&_laserR,&_LSRROn,ADDR_LSR_R,LSRStr);
 
    // Activate third laser sensor
    _toSend = B00001110;
    LSRStr = 'A'; 
    _initLSR(_toSend,&_laserA,&_LSRAOn,ADDR_LSR_A,LSRStr);

    // Activate fourth laser sensor
    _toSend = B00011110;
    LSRStr = 'U'; 
    _initLSR(_toSend,&_laserU,&_LSRUOn,ADDR_LSR_U,LSRStr);

    // Activate fifth laser sensor
    _toSend = B00111110;
    LSRStr = 'D'; 
    _initLSR(_toSend,&_laserD,&_LSRDOn,ADDR_LSR_D,LSRStr);
}

//-----------------------------------------------------------------------------
void CollisionManager::_sendByteWithI2C(byte inByte){
    Wire.beginTransmission(ADDR_FOLLBOARD);
    Wire.write(inByte);
    Wire.endTransmission();
}


