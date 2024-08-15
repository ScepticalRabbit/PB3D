//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef PB3D_CONSTANTS
#define PB3D_CONSTANTS

//------------------------------------------------------------------------------
// Mood
enum EMoodCode{
    MOOD_TEST = 0,
    MOOD_NEUTRAL,
    MOOD_HAPPY,
    MOOD_SAD,
    MOOD_ANGRY,
    MOOD_SCARED,
    MOOD_COUNT
};

//------------------------------------------------------------------------------
// Tasks
enum ETaskCode{
    TASK_TEST = -7,
    TASK_PAUSE = -4,     // Only called by other tasks
    TASK_PICKEDUP = -3,  // Only called by other tasks
    TASK_INTERACT = -2,  // Only called by other tasks
    TASK_TANTRUM = -1,   // Only called by other tasks
    TASK_EXPLORE = 0,
    TASK_REST = 1,
    TASK_DANCE = 2,
    TASK_FINDHUMAN = 3,
    TASK_FINDSOUND = 4,
    TASK_FINDLIGHT = 5,
    TASK_FINDDARK = 6,
    TASK_POUNCE = 7,
};

//------------------------------------------------------------------------------
// Move

// MOVEMENT CONTROL
enum EMoveControl{
    MOVE_CONTROL_POWER = 0,
    MOVE_CONTROL_SPEED = 1
};

enum EMoveEscapeCode{
    MOVE_E_RANDDIR = -1,
    MOVE_E_NOREV = -2,
};

// BASIC movement codes
enum EMoveBasic{
    MOVE_B_STOP = -0,
    MOVE_B_FORCEUPD = -1,
    MOVE_B_FORWARD = 0,
    MOVE_B_BACK = 1,
    MOVE_B_LEFT = 3,
    MOVE_B_RIGHT = 4,
    MOVE_B_FORLEFT = 5,
    MOVE_B_FORRIGHT = 6,
    MOVE_B_BACKLEFT = 7,
    MOVE_B_BACKRIGHT = 8,
    MOVE_B_TODIST_CPOS = 9,
    MOVE_B_TODIST_CSpeed = 10,
    MOVE_B_TOANG_CPOS = 11,
    MOVE_B_TOANG_CSpeed = 12,
    MOVE_B_COUNT = 13,
};

enum EMoveCompound{
    MOVE_C_ESCAPE = -1,
    MOVE_C_STRAIGHT = 0,
    MOVE_C_ZIGZAG = 1,
    MOVE_C_CIRCLE = 2,
    MOVE_C_SPIRAL = 3,
    MOVE_C_LOOK = 4,
    MOVE_C_COUNT = 5,
};

//------------------------------------------------------------------------------
// Collision
enum EDangerCode{
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

//------------------------------------------------------------------------------
// Sensors: Lasers
enum ELaserIndex{
    LASER_CENTRE = 0,
    LASER_UP_CENTRE,
    LASER_DOWN_LEFT,
    LASER_DOWN_RIGHT,
    LASER_HALF_LEFT,
    LASER_HALF_RIGHT,
    LASER_LEFT,
    LASER_RIGHT,
    LASER_BACK,
    LASER_ALT,
    LASER_COUNT
};

#endif