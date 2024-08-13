//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef DANGER_FLAGS
#define DANGER_FLAGS

enum EDangerFlag{
    DANGER_NONE = 0,
    DANGER_SLOW,
    DANGER_FAR,
    DANGER_CLOSE,
    DANGER_COUNT
};

enum EBumpCode{
    BUMP_LEFT = 0,
    BUMP_RIGHT,
    BUMP_REAR,
    BUMP_COUNT
};

#endif