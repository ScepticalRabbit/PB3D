//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef ENCODER_CONTROL_STATE_H
#define ENCODER_CONTROL_STATE_H

#include <Arduino.h>

class EncoderControlState{
public:
    EncoderControlState(){}

    void start_with_end(int32_t start_left, int32_t start_right,
                         int32_t end_left, int32_t end_right){
        _start_count_left = start_left;
        _start_count_right = start_right;
        _end_count_left = end_left;
        _end_count_right = end_right;
        _diff_to_end_left = end_left - start_left;
        _diff_to_end_right = end_right - start_right;
    }

    void start_with_diff(int32_t start_left, int32_t start_right,
                         int32_t diff_left, int32_t diff_right){
        _start_count_left = start_left;
        _start_count_right = start_right;
        _diff_to_end_left = diff_left;
        _diff_to_end_right = diff_right;
        _end_count_left = start_left+diff_left;
        _end_count_right = start_right+diff_right;
    }

    int32_t end_count_left(){return _end_count_left;}
    int32_t end_count_right(){return _end_count_right;}

    int32_t counts_from_start_left(int32_t count){return count - _start_count_left;}
    int32_t counts_from_start_right(int32_t count){return count - _start_count_right;}

    int32_t get_counts_to_end_left(){return _diff_to_end_left;}
    int32_t get_counts_to_end_right(){return _diff_to_end_right;}

    void reset(){
        _start_count_left = 0;
        _end_count_left = 0;
        _diff_to_end_left = 0;
        _start_count_right = 0;
        _end_count_right = 0;
        _diff_to_end_right = 0;
    }

private:
    int32_t _start_count_left = 0;
    int32_t _end_count_left = 0;
    int32_t _diff_to_end_left = 0;

    int32_t _start_count_right = 0;
    int32_t _end_count_right = 0;
    int32_t _diff_to_end_right = 0;
};

#endif