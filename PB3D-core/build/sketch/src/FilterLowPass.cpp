#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/FilterLowPass.cpp"
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

FilterLowPass::FilterLowPass(double alpha){
    _alpha = alpha;
}

FilterLowPass::FilterLowPass(double alpha, uint16_t update_time){
    _alpha = alpha;
    _update_time = update_time;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void FilterLowPass::begin(){
    _filter_timer.start(0);
}

//---------------------------------------------------------------------------
// FILTER
double FilterLowPass::filter(double data){
    if(_filter_timer.finished()){
        _curr_filtered = _alpha*data + (1-_alpha)*_prev_filtered;
        _prev_filtered = _curr_filtered;
        _filter_timer.start(_update_time);
    }
    return _curr_filtered;
}