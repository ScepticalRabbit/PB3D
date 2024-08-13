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
    _update_time = inUpdateTime;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void FilterLowPass::begin(){
    _filter_timer.start(0);
}

//---------------------------------------------------------------------------
// FILTER
double FilterLowPass::filter(double inData){
    if(_filter_timer.finished()){
        _curr_filtered = _alpha*inData + (1-_alpha)*_prev_filtered;
        _prev_filtered = _curr_filtered;
        _filter_timer.start(_update_time);
    }
    return _curr_filtered;
}