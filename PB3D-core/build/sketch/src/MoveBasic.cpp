#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/MoveBasic.cpp"
//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TEMPLATE
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "MoveBasic.h"

//---------------------------------------------------------------------------
// CONSTRUCTOR: pass in pointers to main objects and other sensors
//---------------------------------------------------------------------------
MoveBasic::MoveBasic(){

} 

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
//---------------------------------------------------------------------------
void MoveBasic::begin(){

}

//---------------------------------------------------------------------------
// UPDATE: called during every LOOP
//---------------------------------------------------------------------------
void MoveBasic::update(){
    if(!_isEnabled){return;}


}

//---------------------------------------------------------------------------
// DOSOMETHING - called during the main during decision tree
//---------------------------------------------------------------------------
void MoveBasic::doSomething(){
    if(!_isEnabled){return;}

    if(_startFlag){
        _startFlag = false;
    }

}


