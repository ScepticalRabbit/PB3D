//==============================================================================
// PB3D: A 3D printed pet robot
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
    void circle(EMoveTurn turn_dir);

    EMoveTurn turn_direction = MOVE_TURN_LEFT;

private:
    MoveBasic* _move_basic = NULL;
};
#endif
