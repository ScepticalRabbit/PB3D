//-----------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// CLASS: CollisionManager
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include "CollisionManager.h"

//-----------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//-----------------------------------------------------------------------------
CollisionManager::CollisionManager(MoodManager* inMood, Task* inTask, Move* inMove){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;

    _escaper.setMoveObj(inMove);
}

//-----------------------------------------------------------------------------
// BEGIN: called during SETUP
//-----------------------------------------------------------------------------
void CollisionManager::begin(){
    // Start the ultrasonic timer  
    _ultrasonicTimer.start(0);
    _bumperTimer.start(0);
    _slowDownTimer.start(0);

    _laserManager.begin();
    _bumpers.begin();    
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
    if(_ultrasonicTimer.finished()){  
        _ultrasonicTimer.start(_ultrasonicUpdateTime);
        _ultrasonicRanger.update();
    }

    // COLLISION SENSOR: Run laser ranging
    _laserManager.update();   

    // COLLISION SENSOR: Bumper Switches slaved to Nervous System
    if(_bumperTimer.finished()){
        _bumperTimer.start(_bumperUpdateTime);

        _bumpers.update();

        if(_bumpers.getBumpThresCheck()){
            _moodObj->decMoodScore();
            _bumpers.resetBumpCount();
        }
    }

    // COLLISION DETECTION & DECISION
    // NOTE: this sets the collision flags that control escape behaviour! 
    if(_checkAllTimer.finished()){
        _checkAllTimer.start(_checkAllInt);
        _updateCheckVec(); // Sets collision detection flag

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
    _bumpers.reset();
}


//-----------------------------------------------------------------------------
void CollisionManager::setEscapeStart(){   
    _updateCheckVec();    // Check all collision sensors - used for decision tree
    _updateEscapeDecision();
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
void CollisionManager::_updateCheckVec(){
    // uint8_t _checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
    // NOTE: if's have to have most severe case first!

    // Bumper - Left
    _checkVec[0] = _bumpers.getColCode(0);
    // Bumper - Right
    _checkVec[1] = _bumpers.getColCode(1);

    // Ultrasonic Ranger
    _checkVec[2] = _ultrasonicRanger.getColCode();

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
    _lastCol.USRange = _ultrasonicRanger.getRangeMM();
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
        Serial.println(F("======================================="));
        
        Serial.println(F("CheckVec=[BL,BR,US,LL,LR,LU,LD,]"));
        Serial.print("CheckVec=[");
        for(uint8_t ii=0;ii<_checkNum;ii++){
            Serial.print(" ");Serial.print(_checkVec[ii]);Serial.print(","); 
        }
        Serial.println("]");
        Serial.println();

        Serial.print("US= "); Serial.print(_lastCol.USRange); Serial.println(" mm");
        Serial.print("LL= "); Serial.print(_lastCol.LSRRangeL); Serial.println(" mm");
        Serial.print("LR= " ); Serial.print(_lastCol.LSRRangeR); Serial.println(" mm");
        Serial.print("LU= "); Serial.print(_lastCol.LSRRangeU); Serial.println(" mm");
        Serial.print("LD= "); Serial.print(_lastCol.LSRRangeD); Serial.println(" mm");
        
        Serial.println();
        Serial.print("Esc,Count="); Serial.print(_lastCol.escCount);
        Serial.print(", Dist="); Serial.print(_lastCol.escDist);
        Serial.print(", Angle="); Serial.print(_lastCol.escAng);
        Serial.println();
        
        Serial.println(F("======================================="));
        Serial.println();
    #endif 
}



