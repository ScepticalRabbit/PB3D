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
// BEGIN: called during SETUP
//-----------------------------------------------------------------------------
void CollisionManager::begin(){
    // Start the ultrasonic timer  
    _USTimer.start(0);
    _BMPRTimer.start(0);
    _slowDownTimer.start(0);

    _laserManager.begin();    
}

//-----------------------------------------------------------------------------
// UPDATE: called during LOOP
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
        _updateUSRanger();
    }

    // COLLISION SENSOR: Run laser ranging
    _laserManager.update();   

    // COLLISION SENSOR: Bumper Switches slaved to Nervous System
    if(_BMPRTimer.finished()){
        _BMPRTimer.start(_BMPRUpdateTime);
        _updateBumpers();
    }

    // COLLISION DETECTION & DECISION
    // NOTE: this sets the collision flags that control behaviour! 
    if(_checkAllTimer.finished()){
        _checkAllTimer.start(_checkAllInt);
        _updateCheckVec();

        if(_collisionSlowDown && _slowDownTimer.finished()){
            _slowDownTimer.start(_slowDownInt);
            _moveObj->setSpeedByColFlag(true);
        }
        else if(!_collisionSlowDown && _slowDownTimer.finished()){
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
// Get, set and reset
//--------------------------------------------------------------------------- 
bool CollisionManager::getAltFlag(){
    if(_laserManager.getColCodeA() > DANGER_NONE){
        return true;
    }
    else{
        return false;
    }
}

void CollisionManager::resetFlags(){
    _collisionDetected = false;
    _collisionSlowDown = false;

    _collisionUSFlag = false;
    _collisionBumperFlag = false;
    _collisionNervSys = B00000000;
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

//-----------------------------------------------------------------------------
void CollisionManager::_updateUSRanger(){  
    _USSensRange = _ultrasonicSens->MeasureInCentimeters();
    if(_USSensRange <= _USColDistLim){_USSensRange = 400;}

    if(_USSensRange <= _USColDistClose){
        _collisionDetected = true;
        _collisionUSFlag = true;
    }
    else if(_USSensRange <= _USColDistFar){
        _collisionDetected = true;
        _collisionUSFlag = true;
    }
}

//-----------------------------------------------------------------------------
void CollisionManager::_updateBumpers(){
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
        _collisionDetected = true;
        _collisionBumperFlag = true;
    }
    if ((nervSysFlags & B00000010) == B00000010){
        //Serial.println("Front Right Bumper");
        _collisionDetected = true;
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

//---------------------------------------------------------------------------
void CollisionManager::_updateCheckVec(){
    // uint8_t _checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
    // NOTE: if's have to have most severe case first!

    // Bumpers
    if(((_collisionNervSys & B00000001) == B00000001)){_checkVec[0] = DANGER_CLOSE;}
    else{_checkVec[0] = 0;}
    if(((_collisionNervSys & B00000010) == B00000010)){_checkVec[1] = DANGER_CLOSE;}
    else{_checkVec[1] = 0;}

    // Ultrasonic Ranger
    if(_USSensRange <= _USColDistClose){_checkVec[2] = DANGER_CLOSE;}
    else if(_USSensRange <= _USColDistFar){_checkVec[2] = DANGER_FAR;}
    else if(_USSensRange <= _USColDistSlowD){_checkVec[2] = DANGER_SLOWD;}
    else{_checkVec[2] = 0;}

    // Laser - Left
    _checkVec[3] = _laserManager.getColCodeL();
    // Laser - Right
    _checkVec[4] = _laserManager.getColCodeR();
    // Laser - Up Angle
    _checkVec[5] = _laserManager.getColCodeU();
    // Laser - Down Angle - TODO, fix this so we know which is which
    _checkVec[6] = _laserManager.getColCodeD();

    // If anything is tripped set flags to true
    _collisionDetected = false;
    _collisionSlowDown = false; 
    for(uint8_t ii=0;ii<_checkNum;ii++){
        if(_checkVec[ii] >= DANGER_SLOWD){
            _collisionSlowDown = true;
        }
        if(_checkVec[ii] >= DANGER_FAR){
            _collisionDetected = true;
        }
    }
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
    _lastCol.LSRRangeL = _laserManager.getRangeL();
    _lastCol.LSRRangeR = _laserManager.getRangeR();
    _lastCol.LSRRangeU = _laserManager.getRangeU();
    _lastCol.LSRRangeD = _laserManager.getRangeD();
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



