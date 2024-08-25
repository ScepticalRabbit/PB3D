//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_WIGGLE_H
#define MOVE_WIGGLE_H

#include <Arduino.h>

class MoveWiggle{
public:
    MoveWiggle(){};

    void wiggle();
    void wiggle(uint16_t left_dur, uint16_t right_dur);


private:

};
#endif
