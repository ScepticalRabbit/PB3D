//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_FORWARD_BACK_H
#define MOVE_FORWARD_BACK_H

#include <Arduino.h>
#include "MoveBasic.h"

class MoveForwardBack{
public:
    MoveForwardBack(MoveBasic* move_basic){
        _move_basic_code = move_basic;
    };

    void forward_back();
    void forward_back(uint16_t forward_dur, uint16_t back_dur);

private:
    MoveBasic* _move_basic_code = NULL;

};
#endif
