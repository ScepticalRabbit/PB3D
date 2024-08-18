#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/TaskManager.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <Arduino.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include "MoodManager.h"
#include "Timer.h"


class TaskManager{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  //---------------------------------------------------------------------------
  TaskManager(Adafruit_NeoPixel_ZeroDMA* RGBLEDs);

  //---------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  //---------------------------------------------------------------------------
  void begin();

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  //---------------------------------------------------------------------------
  void update();
  void force_update();

  //---------------------------------------------------------------------------
  // Get, set and reset
  //---------------------------------------------------------------------------
  // Default setTask function uses default task durations
  void setTask(int8_t taskIn);
  void setTaskDuration(uint32_t taskDur);
  void assignProb(int8_t moodIn);

  int8_t getTask(){return _taskCode;}
  bool getNewTaskFlag(){return _taskNewFlag;}
  void setNewTaskFlag(bool inFlag){_taskNewFlag = inFlag;}
  void setDanceDuration(uint32_t inDuration){_danceDuration = inDuration;}
  void setTantrumDuration(uint16_t inDuration){_tantrumDuration = inDuration;}
  void setDanceUpdateFlag(bool inFlag){_danceUpdateFlag = inFlag;}
  bool getDanceUpdateFlag(){return _danceUpdateFlag;}

  //------------------------------------------------------------------------
  // TASK LED FUNCTIONS
  //---------------------------------------------------------------------------
  void taskLEDCollision();
  void taskLEDExplore();
  void taskLEDRest(uint8_t intensity);
  void taskLEDDance();
  void taskLEDTantrum();
  void taskLEDFindHuman();
  void taskLEDInteract();
  void taskLEDPickedUpOk();
  void taskLEDPickedUpPanic();
  void taskLEDPause(uint16_t pauseTime);
  void taskLEDFindLight();
  void taskLEDFindDark();
  void taskLEDFindSound();
  void taskLEDTest();
  void taskLEDOff();

  //------------------------------------------------------------------------
  // HSV LEDS
  void taskLEDHue(uint16_t hue);
  void taskLEDHSV(uint16_t hue, uint8_t sat, uint8_t value);
  void taskLEDCol(uint16_t col);
  void taskLEDCol(uint16_t colL, uint16_t colR);
  void taskLEDCSV(uint16_t col, uint8_t sat, uint8_t val);
  void taskLEDCSV(uint16_t colL,uint16_t colR,uint8_t satL,uint8_t satR, uint8_t valL, uint8_t valR);

private:
  //------------------------------------------------------------------------
  // TASK Private Functions
  void _update(){
    _taskPc = random(0,100); // NOTE: random num between (min,max-1)
    _danceUpdateFlag = false;

    if((_taskPc >= 0) && (_taskPc < _taskProbBounds[0])){ // EXPLORE
      setTask(TASK_EXPLORE);
    }
    else if((_taskPc >= _taskProbBounds[0]) && (_taskPc < _taskProbBounds[1])){ // REST
      setTask(TASK_REST);
    }
    else if((_taskPc >= _taskProbBounds[1]) && (_taskPc < _taskProbBounds[2])){ // DANCE
      _danceUpdateFlag = true;
      setTask(TASK_DANCE);
    }
    else if((_taskPc >= _taskProbBounds[2]) && (_taskPc < _taskProbBounds[3])){ // FINDHUMAN
      setTask(TASK_FINDHUMAN);
    }
    else if((_taskPc >= _taskProbBounds[3]) && (_taskPc < _taskProbBounds[4])){ // FINDSOUND
      setTask(TASK_FINDSOUND);
    }
    else if((_taskPc >= _taskProbBounds[4]) && (_taskPc < _taskProbBounds[5])){ // FINDLIGHT
      setTask(TASK_FINDLIGHT);
    }
    else if((_taskPc >= _taskProbBounds[5]) && (_taskPc < _taskProbBounds[6])){ // FINDDARK
      setTask(TASK_FINDDARK);
    }
    else{ // EXPLORE
      setTask(TASK_EXPLORE);
    }
    // Start the timer.
    _taskTimer.start(_taskDuration);
  }

  void _setTaskProb(int8_t inProbs[]){
    int16_t probSum = 0;
    for(int8_t ii = 0; ii < _taskCount; ii++){
      probSum = probSum+inProbs[ii];
      _taskProbBounds[ii] = probSum;
    }
  }

  uint8_t _calcFallingLEDVal(uint16_t timeInt){
    float startVal = 255.0, endVal = 0.0;
    float slope = (float(startVal)-float(endVal))/(float(0.0)-float(timeInt));
    return round(float(startVal) + slope*float(_LEDTimer.get_time()));
  }

  uint8_t _calcRisingLEDVal(uint16_t timeInt){
    float startVal = 0.0,  endVal = 255.0;
    float slope = (float(startVal)-float(endVal))/(float(0.0)-float(timeInt));
    return round(float(startVal) + slope*float(_LEDTimer.get_time()));
  }

  //------------------------------------------------------------------------
  // TASK Variables
  // [1.explore,2.rest,3.dance,4.findhuman,5.findsound,6.findlight,7.finddark]
  int8_t _taskCode = 0;
  int8_t _taskPc = 0;

  const static int8_t _taskCount = 7;
  int8_t _taskProbBounds[_taskCount] = {30,40,55,70,85,95,100};

  int8_t _taskProbTest[_taskCount] =    {100,0,0,0,0,0,0};
  int8_t _taskProbNeutral[_taskCount] = {30,10,15,15,15,10,5};
  int8_t _taskProbHappy[_taskCount] =   {25,5,20,20,15,15,0};
  int8_t _taskProbSad[_taskCount] =     {40,20,5,5,10,0,20};
  int8_t _taskProbAngry[_taskCount] =   {40,0,15,15,15,10,5};
  int8_t _taskProbScared[_taskCount] =  {30,5,10,10,10,0,35};

  uint32_t _taskDuration = 7000;
  uint32_t _taskDurationMin = 5000;
  uint32_t _taskDurationMax = 15000;
  bool _taskNewFlag = true;

  // Sub task variables
  bool _danceUpdateFlag = false;
  uint32_t _danceDuration = 0;
  uint16_t _tantrumDuration = 0;

  // Colours for Tasks
  uint16_t _huePounce = (65536 * 1)/12;

  // Time for tasks
  Timer _taskTimer = Timer();

  // TASK LED Variables
  Timer _LEDTimer = Timer();
  bool _LEDSwitch = true;
  uint16_t _LEDOnOffTime = 500;
  uint16_t _LEDSlopeTime = 1000;
  // Pointer to the mood and task expression LEDs
  Adafruit_NeoPixel_ZeroDMA* _taskLEDs;
};
#endif // TASK_H
