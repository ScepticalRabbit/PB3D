//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TEMPLATE
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "ClassTemplate.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//---------------------------------------------------------------------------
ClassTemp::ClassTemp(Collision* inCollision, MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, 
            Speaker* inSpeaker){
_collisionObj = inCollision;
_moodObj = inMood;
_taskObj = inTask;
_moveObj = inMove;
_speakerObj = inSpeaker;
}

//---------------------------------------------------------------------------
// BEGIN: called during SETUP
//---------------------------------------------------------------------------
void ClassTemp::begin(){

}

//---------------------------------------------------------------------------
// UPDATE: called during LOOP
//---------------------------------------------------------------------------
void ClassTemp::update(){
    if(!_isEnabled){return;}

    if(_taskObj->getNewTaskFlag()){
        _startFlag = true;
    }

}

//---------------------------------------------------------------------------
// DOSOMETHING - called during the main during decision tree
//---------------------------------------------------------------------------
void ClassTemp::doSomething(){
    if(!_isEnabled){return;}

    if(_startFlag){
        _startFlag = false;
    }

}


