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
ClassTemplate::ClassTemplate(Collision* inCollision, Mood* inMood, Task* inTask, Move* inMove, 
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
void ClassTemplate::begin(){

}

//---------------------------------------------------------------------------
// UPDATE: called during LOOP
//---------------------------------------------------------------------------
void ClassTemplate::update(){
    if(!_isEnabled){return;}

    if(_taskObj->getNewTaskFlag()){
        _startFlag = true;
    }

}

//---------------------------------------------------------------------------
// DOSOMETHING - called during the main during decision tree
//---------------------------------------------------------------------------
void ClassTemplate::doSomething(){
    if(!_isEnabled){return;}

    if(_startFlag){
        _startFlag = false;
    }

}


