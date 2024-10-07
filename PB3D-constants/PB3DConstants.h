//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef PB3D_CONSTANTS_H
#define PB3D_CONSTANTS_H

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
enum EMoveBasic{
    MOVE_B_STOP = -2,
    MOVE_B_FORCEUPD = -1,
    MOVE_B_FORWARD = 0,
    MOVE_B_BACK = 1,
    MOVE_B_LEFT = 2,
    MOVE_B_RIGHT = 3,
    MOVE_B_FORLEFT = 4,
    MOVE_B_FORRIGHT = 5,
    MOVE_B_BACKLEFT = 6,
    MOVE_B_BACKRIGHT = 7,
    MOVE_B_TODIST_CPOS = 8,
    MOVE_B_TODIST_CSpeed = 9,
    MOVE_B_TOANG_CPOS = 10,
    MOVE_B_TOANG_CSpeed = 11,
    MOVE_B_COUNT = 12,
};

enum EMoveCompound{
    MOVE_C_WIGGLE = -3,
    MOVE_C_FORWARDBACK = -2,
    MOVE_C_ESCAPE = -1,
    MOVE_C_STRAIGHT = 0,
    MOVE_C_ZIGZAG = 1,
    MOVE_C_CIRCLE = 2,
    MOVE_C_SPIRAL = 3,
    MOVE_C_LOOK = 4,
    MOVE_C_COUNT = 5,
};

enum EMoveTurn{
    MOVE_TURN_LEFT = 0,
    MOVE_TURN_RIGHT = 1,
    MOVE_TURN_COUNT = 2,
};

enum EMoveZigZag{
    ZIGZAG_STRAIGHT = 0,
    ZIGZAG_TURN,
};

enum EMoveControlMode{
    MOVE_MODE_POWER = 0,
    MOVE_MODE_SPEED = 1
};

enum EMoveControlState{
    MOVE_CONTROL_START = 0,
    MOVE_CONTROL_INCOMPLETE,
    MOVE_CONTROL_LEFT_COMPLETE,
    MOVE_CONTROL_RIGHT_COMPLETE,
    MOVE_CONTROL_COMPLETE,
    MOVE_CONTROL_TIMEOUT,
};

enum EMoveEscapeCode{
    MOVE_E_RANDDIR = -1,
    MOVE_E_NOREV = -2,
};

enum EMoveLookState{
    MOVE_LOOK_START = 0,
    MOVE_LOOK_PAUSE,
    MOVE_LOOK_MOVING,
    MOVE_LOOK_COMPLETE,
};


//------------------------------------------------------------------------------
// Motors
enum EMotorNums{
    MOTOR_LEFT = 2,
    MOTOR_RIGHT = 1,
};

//------------------------------------------------------------------------------
// Encoders
enum EEncoderDirection{
    ENCODER_FORWARD = 0,
    ENCODER_BACK
};

//------------------------------------------------------------------------------
// PIDs
enum EPIDCode{
    PID_OFF = 0,
    PID_ON,
};

enum EPIDCommand{
    PID_DIRECT = 0,
    PID_REVERSE,
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

enum EEscapeState{
    ESC_STATE_BEGIN = 0,
    ESC_STATE_REV,
    ESC_STATE_TURN,
    ESC_STATE_COMPLETE,
};

enum ECollisionStrategy{
    AVOID_BASIC = 0,
    AVOID_FLAT_SLOW,
    AVOID_OVERHEAD,
    AVOID_CLIFF,
    AVOID_PICKUP,
    AVOID_SLOW_ONLY,
};

//------------------------------------------------------------------------------
// LEDs
enum ELEDs{
    NUM_PIX = 4
};

//------------------------------------------------------------------------------
// Sensors: Bumpers
enum EBumpCode{
    BUMP_LEFT = 0,
    BUMP_RIGHT,
    BUMP_BACK,
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

const static char* LASER_STRS[] = {"CC",
                                   "UC",
                                   "DL",
                                   "DR",
                                   "HL",
                                   "HR",
                                   "LL",
                                   "RR",
                                   "BB",
                                   "AA"};


//------------------------------------------------------------------------------
// Light Sensors through TCA Multiplexer
enum ELightSens{
    LIGHTSENS_L = 1,
    LIGHTSENS_R = 0,
};

//------------------------------------------------------------------------------
// SPEAKER
enum ESpeakerCode{
    SPEAKER_OFF = 0,
    SPEAKER_BEEP = 1,
    SPEAKER_SLIDE = 2,
    SPEAKER_SNORE = 3,
    SPEAKER_GROWL = 4,
    SPEAKER_SCREECH = 5,
    SPEAKER_BABBLE = 6,
};

enum ESoundStatus{
    SOUND_PLAY = 0,
    SOUND_PAUSE = 1,
    SOUND_END = 2,
};

//------------------------------------------------------------------------------
// TAIL
enum ETailCode{
    TAIL_CENT = 0,
    TAIL_SET_POS,
    TAIL_WAG_CON,
    TAIL_WAG_INT,
};

//------------------------------------------------------------------------------
// TASK: Dance
enum EDanceCode{
    DANCE_STOP = 0,
    DANCE_WIGGLE,
    DANCE_FORBACK,
    DANCE_CIRCLE,
    DANCE_SPIN,
    DANCE_TURN,
    DANCE_NUM_MOVES
};

//------------------------------------------------------------------------------
// TASK: Pounce
enum EPounce{
    POUNCE_SEEK = 0,
    POUNCE_LOCKON,
    POUNCE_RUN,
    POUNCE_REALIGN,
};

//------------------------------------------------------------------------------
// TASK: Find light
enum EFindLight{
    SEEK_LIGHT = 0,
    SEEK_DARK,
};

//------------------------------------------------------------------------------
// TASK: Find sound
enum EEarCode{
    EAR_COM_NOSOUND = 0,
    EAR_COM_FORWARD,
    EAR_COM_LEFT,
    EAR_COM_RIGHT,
    EAR_COM_SENV,
};

//------------------------------------------------------------------------------
// Musical Notes for Sound Generation
enum EMusicNote{
    NOTE_C1 = 33,
    NOTE_CS1= 35,
    NOTE_D1 = 37,
    NOTE_DS1= 39,
    NOTE_E1 = 41,
    NOTE_F1 = 44,
    NOTE_FS1= 46,
    NOTE_G1 = 49,
    NOTE_GS1= 52,
    NOTE_A1 = 55,
    NOTE_AS1= 58,
    NOTE_B1 = 62,
    NOTE_C2 = 65,
    NOTE_CS2= 69,
    NOTE_D2 = 73,
    NOTE_DS2= 78,
    NOTE_E2 = 82,
    NOTE_F2 = 87,
    NOTE_FS2= 93,
    NOTE_G2 = 98,
    NOTE_GS2= 104,
    NOTE_A2 = 110,
    NOTE_AS2= 117,
//NOTE: analog out for speaker seems to cause errors below 120Hz
    NOTE_B2 = 123,
    NOTE_C3 = 131,
    NOTE_CS3= 139,
    NOTE_D3 = 147,
    NOTE_DS3= 156,
    NOTE_E3 = 165,
    NOTE_F3 = 175,
    NOTE_FS3= 185,
    NOTE_G3 = 196,
    NOTE_GS3= 208,
    NOTE_A3 = 220,
    NOTE_AS3= 233,
    NOTE_B3 = 247,
    NOTE_C4 = 262,
    NOTE_CS4= 277,
    NOTE_D4 = 294,
    NOTE_DS4= 311,
    NOTE_E4 = 330,
    NOTE_F4 = 349,
    NOTE_FS4= 370,
    NOTE_G4 = 392,
    NOTE_GS4= 415,
    NOTE_A4 = 440,
    NOTE_AS4= 466,
    NOTE_B4 = 494,
    NOTE_C5 = 523,
    NOTE_CS5= 554,
    NOTE_D5 = 587,
    NOTE_DS5= 622,
    NOTE_E5 = 659,
    NOTE_F5 = 698,
    NOTE_FS5= 740,
    NOTE_G5 = 784,
    NOTE_GS5= 831,
    NOTE_A5 = 880,
    NOTE_AS5= 932,
    NOTE_B5 = 988,
    NOTE_C6 = 1047,
    NOTE_CS6= 1109,
    NOTE_D6 = 1175,
    NOTE_DS6= 1245,
    NOTE_E6 = 1319,
    NOTE_F6 = 1397,
    NOTE_FS6= 1480,
    NOTE_G6 = 1568,
    NOTE_GS6= 1661,
    NOTE_A6 = 1760,
    NOTE_AS6= 1865,
    NOTE_B6 = 1976,
    NOTE_C7 = 2093,
    NOTE_CS7= 2217,
    NOTE_D7 = 2349,
    NOTE_DS7= 2489,
    NOTE_E7 = 2637,
    NOTE_F7 = 2794,
    NOTE_FS7= 2960,
    NOTE_G7 = 3136,
    NOTE_GS7= 3322,
    NOTE_A7 = 3520,
    NOTE_AS7= 3729,
    NOTE_B7 = 3951,
    NOTE_C8 = 4186,
};

#endif