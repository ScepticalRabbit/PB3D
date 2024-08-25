//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef MOVE_CIRCLE_H
#define MOVE_CIRCLE_H

#include <Arduino.h>
#include <PB3DConstants.h>
#include "MoveBasic.h"

class MoveCircle{
public:
    MoveCircle(MoveBasic* move_basic){
        _move_basic = move_basic;
    };

    void circle();
    void circle(int8_t turn_dir);

    void set_turn_dir(EMoveBasic turn_dir){_turn_direction = turn_dir;}

private:
    MoveBasic* _move_basic = NULL;
    EMoveBasic _turn_direction = MOVE_B_LEFT;

};
#endif
