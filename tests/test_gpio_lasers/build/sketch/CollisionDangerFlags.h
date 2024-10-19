#line 1 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/CollisionDangerFlags.h"
//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef DANGER_FLAGS
#define DANGER_FLAGS

enum EDangerCode{
    DANGER_NONE = 0,
    DANGER_SLOW,
    DANGER_FAR,
    DANGER_CLOSE
};

#endif