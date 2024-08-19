#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/FilterMovAvg.cpp"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "FilterMovAvg.h"

FilterMovAvg::FilterMovAvg(){
    _dataArray = new double[_window];
    for(uint8_t ii=0 ; ii<_window ; ii++){
        _dataArray[ii] = 0.0;
    }
}

FilterMovAvg::FilterMovAvg(uint8_t window){
    _window = window;
    _dataArray = new double[window];
    for(uint8_t ii=0 ; ii<window ; ii++){
        _dataArray[ii] = 0.0;
    }
}

FilterMovAvg::FilterMovAvg(uint8_t window, uint16_t inUpdateTime){
    _window = window;
    _update_time = inUpdateTime;
    _dataArray = new double[window];
    for(uint8_t ii=0 ; ii<window ; ii++){
        _dataArray[ii] = 0.0;
    }
}

FilterMovAvg::~FilterMovAvg(){
    delete []_dataArray;
}

//---------------------------------------------------------------------------
// BEGIN: called once during SETUP
void FilterMovAvg::begin(){
    _filter_timer.start(0);
}

//---------------------------------------------------------------------------
// FILTER
double FilterMovAvg::filter(double inData){
    if(_filter_timer.finished()){
        // Remove the data point at the current position from the running total
        _dataSum = _dataSum - _dataArray[_currIndex];
        // Overwrite the data point at the current position
        _dataArray[_currIndex] = inData;
        // Add the current data point to the running total
        _dataSum = _dataSum + inData;
        // Increment the index and wrap around if needed
        _currIndex++;
        if(_currIndex >= _window){
        _currIndex = 0;
        }
        // The filtered value is the average over our window, return it
        _curr_filtered = _dataSum / _window;

        _filter_timer.start(_update_time);
    }
    return _curr_filtered;
}

//---------------------------------------------------------------------------
// Get, set and reset
void FilterMovAvg::reset(){
    for(uint8_t ii=0 ; ii<_window ; ii++){
        _dataArray[ii] = 0.0;
    }
}

void FilterMovAvg::reset(float inVal){
    for(uint8_t ii=0 ; ii<_window ; ii++){
        _dataArray[ii] = inVal;
    }
}