//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

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