//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS: TaskDance
//---------------------------------------------------------------------------
/*
The task X class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef TASKDANCE_H
#define TASKDANCE_H

#include <Arduino.h>
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"

#define DANCE_STOP 0
#define DANCE_WIGGLE 1
#define DANCE_FORBACK 2
#define DANCE_CIRCLE 3
#define DANCE_SPIN 4
#define DANCE_TURN 5
// NOTE: increment this for random generator
#define DANCE_NUM_MOVES 6

//---------------------------------------------------------------------------
class TaskDance{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskDance(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, 
            Speaker* inSpeaker);

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every loop
  void update();

  //---------------------------------------------------------------------------
  // DANCE
  void dance();

  //---------------------------------------------------------------------------
  // Get, set and reset
  float getDanceBarMs(){return _danceBarMs;}
  uint32_t getDuration(){return _danceDuration;}
  void setStartFlag(bool inFlag){_danceStartFlag = inFlag;}
  bool getStartFlag(){return _danceStartFlag;}
  bool getSpeakerFlag(){return _speakerFlag;}
  void setSpeakerFlag(bool inFlag){_speakerFlag = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  void _startDance();
  void _generateTempo();
  void _generateDance();
  void _updateDance();
  
  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  MoodManager* _moodObj;
  TaskManager* _taskObj;
  MoveManager* _moveObj;
  Speaker* _speakerObj;

  //---------------------------------------------------------------------------
  // TASK - DANCE Variables
  // Timers
  Timer _timerObj = Timer();
  // Need to work out how to create complex move sets with pauses
  // Wiggle left, Stop, Wiggle Right, Stop etc
  float _danceBPM = 140.0;
  float _dance4NoteMs = (60.0/_danceBPM)*1000.0;
  float _danceBarMs = _dance4NoteMs*4.0;
  uint8_t _danceNumBars = 8;
  uint32_t _danceDuration = uint32_t(_danceBarMs*float(_danceNumBars));
  
  int16_t _danceBPMMin = 120, _danceBPMMax = 180;
  uint8_t _danceCurrMove = 0;
  uint8_t _danceCurrBar = 0;
  uint8_t _danceMoveInd = 0;
  uint8_t _danceNumMoves = 8;
  uint8_t _danceMoveVec[8] = {1,3,1,3,1,3,1,3};
  bool _danceStartFlag = true;

  int8_t _danceTurnDir = MOVE_B_LEFT;
  int8_t _danceSpinDir = MOVE_B_LEFT;
  
  bool _speakerFlag = false;
};
#endif // TASKDANCE
