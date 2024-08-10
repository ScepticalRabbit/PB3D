//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "CollisionEscaper.h"
#include "CollisionDangerFlags.h"

//-----------------------------------------------------------------------------
void CollisionEscaper::setMoveObj(MoveManager* inMove){
    _moveObj = inMove;
}

//-----------------------------------------------------------------------------
void CollisionEscaper::updateEscapeDecision(uint8_t checkVec[]){
    // _checkVec[7] = {BL,BR,US,LL,LR,LU,LD}
    // NOTE: first thing in the tree is dealt with first

    // 1) BMPL: reverse a full body length and turn +90 deg away
    if(checkVec[0] == DANGER_CLOSE){
        _escapeCount = ESCAPE_REV;
        _escapeDist = 1.0*_defRevDist;
        _escapeAngle = -1.0*_defHardTurn;
    }
    // 2) BMPR: reverse a full body length and turn -90 deg away
    else if(checkVec[1] == DANGER_CLOSE){
        _escapeCount = ESCAPE_REV;
        _escapeDist = 1.0*_defRevDist;
        _escapeAngle = 1.0*_defHardTurn;
    }
    // 7) LSRD: check cliff-close/far, check obstacle-close/far
    else if(checkVec[6] >= DANGER_FAR){
        if(checkVec[6] == DANGER_CLOSE){
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
    else if(checkVec[3] >= DANGER_FAR){
        if(checkVec[3] == DANGER_CLOSE){
        _escapeCount = ESCAPE_REV;
        _escapeDist = 0.8*_defRevDist;
        _escapeAngle = -1.25*_defModTurn;
        }
        else{
        _escapeCount = ESCAPE_NOREV;
        _escapeDist = 0.0;
        _escapeAngle = -1.0*_defModTurn;
        }

        if(checkVec[4] >= DANGER_FAR){ // If the right laser is tripped as well then we are in a corner, do a 180
        _escapeAngle = -180.0 + _getRandTurnDir()*float(random(0,45));
        }
        else if(checkVec[2] >= DANGER_FAR){ // If the ultrasonic sensor is tripped as well then turn harder
        _escapeAngle = -1.0*_defHardTurn;
        }
    }
    // 5) LSRR: check far (norev,-45deg), check close (rev,-90deg)
    else if(checkVec[4] >= DANGER_FAR){
        if(checkVec[4] == DANGER_CLOSE){
        _escapeCount = ESCAPE_REV;
        _escapeDist = 0.8*_defRevDist;
        _escapeAngle = 1.25*_defModTurn;
        }
        else{
        _escapeCount = ESCAPE_NOREV;
        _escapeDist = 0.0;
        _escapeAngle = 1.0*_defModTurn;
        }

        if(checkVec[3] >= DANGER_FAR){ // If the right laser is tripped as well then we are in a corner, do a 180
        _escapeAngle = 180.0 + _getRandTurnDir()*float(random(0,45));
        }
        else if(checkVec[2] >= DANGER_FAR){ // If the ultrasonic sensor is tripped as well then turn harder
        _escapeAngle = -1.0*_defHardTurn;
        }
    }
    // 3) US: check far (norev,+/-45deg), check close (rev,+/-90deg)
    else if(checkVec[2] >= DANGER_FAR){
        if(checkVec[2] == DANGER_CLOSE){
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
    else if(checkVec[5] >= DANGER_FAR){
        if(checkVec[5] == DANGER_CLOSE){
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
}


//-----------------------------------------------------------------------------
void CollisionEscaper::setEscapeStart(uint8_t checkVec[]){
    updateEscapeDecision(checkVec);  // This is the decision tree
}

//-----------------------------------------------------------------------------
void CollisionEscaper::escape(){
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

//-----------------------------------------------------------------------------
bool CollisionEscaper::getEscapeFlag(){
    bool outFlag = false;
    if(_escapeCount <= 1){
        outFlag = true;
    }
    return outFlag;
}

//-----------------------------------------------------------------------------
int8_t CollisionEscaper::getEscapeTurn(uint8_t checkVec[]){
    //_updateCheckVec();    // Check all collision sensors - used for decision tree
    updateEscapeDecision(checkVec);  // This is the decision tree
    if(_escapeAngle > 0.0){
        return MOVE_B_LEFT;
    }
    else{
        return MOVE_B_RIGHT;
    }
}

//-----------------------------------------------------------------------------
float CollisionEscaper::_getRandTurnDir(){
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


