//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS: TaskFindSound
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
*/

#ifndef TASKFINDSOUND_H
#define TASKFINDSOUND_H

#include <Wire.h> // I2C 
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"

// Define follower Xiao I2C Address
#ifndef ADDR_FOLLBOARD
  #define ADDR_FOLLBOARD 0x11
#endif

// Communication flags from sub-board with 'ears'
#define EAR_COM_NOSOUND 0
#define EAR_COM_FORWARD 1
#define EAR_COM_LEFT 2
#define EAR_COM_RIGHT 3
#define EAR_COM_SENV 4

// Debug flag to print debug info to serial
//#define DEBUG_TASKFINDSOUND

class TaskFindSound{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskFindSound(MoodManager* inMood, TaskManager* inTask, 
                MoveManager* inMove, Speaker* inSpeaker);

  //---------------------------------------------------------------------------
  // BEGIN: called during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during each loop to update variables
  void update();

  //---------------------------------------------------------------------------
  // FINDSOUND - called during task decision tree
  void findSound();

  //---------------------------------------------------------------------------
  // GET/SET FUNCTIONS
  bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}
  byte getEarState(){return _earState;}
  void setEarState(byte inState){_earState = inState;}
  byte getSendByte(){return _sendByte;}
  void setSendByte(byte inByte){_sendByte = inByte;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  void _I2CSendByte();
  void _I2CSendSampEnvFlag();
  void _I2CReadEarState();

  //---------------------------------------------------------------------------
  // CLASS VARIABLES
  // Main Object Pointers
  MoodManager* _moodObj = NULL;
  TaskManager* _taskObj = NULL;
  MoveManager* _moveObj = NULL;
  Speaker* _speakerObj = NULL;

  // Base Varibles
  bool _isEnabled = true;
  bool _startFlag = false;

  // TASK - FIND SOUND VARS
  byte _earState = 0;
  byte _sendByte = B00000000;
  float _speedDiffLR = 75.0;
  float _speedDiffFracLR = 0.75;

  // Timer for controlling how often the sensor is sampled
  Timer _sensUpdateTimer = Timer();
  uint16_t _sensUpdateTime = 500;

  Timer _clapEnableTimer = Timer();
  uint16_t _clapEnableUpdateTime = 6000;
  uint8_t _clapThres = 3, _clapCount = 0;

  Timer _envSampTimer = Timer();
  uint16_t _envSampUpdateTime = 20000;

  // Variables for controlling calling: where are you?
  Timer _callTimer = Timer();
  uint16_t _callInterval = 4000;
};
#endif // FINDSOUND
