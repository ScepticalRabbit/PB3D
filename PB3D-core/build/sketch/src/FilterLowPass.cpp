#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/FilterLowPass.cpp"
//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: FilterLowPass
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/
#include "FilterLowPass.h"

 //---------------------------------------------------------------------------
// CONSTRUCTORS
FilterLowPass::FilterLowPass(){
}

FilterLowPass::FilterLowPass(double inAlpha){
    _alpha = inAlpha;
}

FilterLowPass::FilterLowPass(double inAlpha, uint16_t inUpdateTime){
    _alpha = inAlpha;
    _updateTime = inUpdateTime;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void FilterLowPass::begin(){
    _filtTimer.start(0);
}

//---------------------------------------------------------------------------
// FILTER
double FilterLowPass::filter(double inData){
    if(_filtTimer.finished()){
        _currFiltered = _alpha*inData + (1-_alpha)*_prevFiltered;
        _prevFiltered = _currFiltered;
        _filtTimer.start(_updateTime);
    }
    return _currFiltered;
}