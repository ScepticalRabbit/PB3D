#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskFindHuman.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKFINDHUMAN_H
#define TASKFINDHUMAN_H

#include <Wire.h> // I2C
#include "Grove_Human_Presence_Sensor.h" // IR presence sensor
#include "MoodManager.h"
#include "TaskManager.h"
#include "MoveManager.h"
#include "Speaker.h"
#include "Timer.h"
#include "TaskInteract.h"

class TaskFindHuman{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  TaskFindHuman(MoodManager* inMood, TaskManager* inTask, MoveManager* inMove, Speaker* inSpeaker,
                TaskInteract* inTInt);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();

  //---------------------------------------------------------------------------
  // FIND HUMAN - called during task decision tree
  void findHuman();

  //---------------------------------------------------------------------------
  // Get, set and reset
   bool getEnabledFlag(){return _isEnabled;}
  void setEnabledFlag(bool inFlag){_isEnabled = inFlag;}

private:
  //---------------------------------------------------------------------------
  void _sensUpdateHumanIRSensor();

  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  MoodManager* _moodObj = NULL;
  TaskManager* _taskObj = NULL;
  MoveManager* _moveObj = NULL;
  Speaker* _speakerObj = NULL;
  TaskInteract* _taskInteractObj = NULL;

  // Timers
  Timer _callTimer = Timer();

  // TASK - FIND HUMAN
  bool _isEnabled = true;
  bool _startFlag = false;

  // TASK - FIND HUMAN - Grove Human Sensor
  AK9753 _movementSensor;
  float _sensitivityPresence = 12.0;
  float _sensitivityMovement = 11.5;
  float _sensitivityTrack = 2.9;
  uint16_t _detectInterval = 30; //milliseconds
  PresenceDetector _presDetector = PresenceDetector(_movementSensor, _sensitivityPresence, _sensitivityMovement, _detectInterval);

  Timer _sensUpdateTimer = Timer();
  uint8_t _humIRSensUpdateTime = 50;
  //NOTE: Sensors are organised clock wise with 1 closest to the grove connector
  bool _IRPFlags1 = false;
  bool _IRPFlags2 = false;
  bool _IRPFlags3 = false;
  bool _IRPFlags4 = false;
  float _IRtDer1 =0.0;
  float _IRtDer2 =0.0;
  float _IRtDer3 =0.0;
  float _IRtDer4 =0.0;
  float _diffIR13=0.0;
  float _diffIR24=0.0;

  // Variables for controlling calling: where you?
  uint16_t _callInterval = 5000;
};
#endif // FINDHUMAN
