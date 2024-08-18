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
  TaskManager(Adafruit_NeoPixel_ZeroDMA* RGBLEDs);

  //----------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin();

  //----------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update();
  void force_update();

  //----------------------------------------------------------------------------
  // Get, set and reset
  // Default setTask function uses default task durations
  void set_task(int8_t task);
  void set_task_duration(uint32_t task_dur);
  void assign_probability(int8_t mood);

  ETaskCode get_task(){return _task_code;}
  bool get_new_task_flag(){return _task_new_flag;}
  void set_new_task_flag(bool flag){_task_new_flag = flag;}
  void set_dance_duration(uint32_t duration){_dance_duration = duration;}
  void set_tantrum_duration(uint16_t duration){_tantrum_duration = duration;}
  void set_dance_update_flag(bool flag){_dance_update_flag = flag;}
  bool get_dance_update_flag(){return _dance_update_flag;}

  //----------------------------------------------------------------------------
  // TASK LED FUNCTIONS
  void task_LED_collision();
  void task_LED_explore();
  void task_LED_rest(uint8_t intensity);
  void task_LED_dance();
  void task_LED_tantrum();
  void task_LED_find_human();
  void task_LED_interact();
  void task_LED_pickedup_Ok();
  void task_LED_pickedup_panic();
  void task_LED_pause(uint16_t pauseTime);
  void task_LED_find_light();
  void task_LED_find_dark();
  void task_LED_find_sound();
  void task_LED_test();
  void task_LED_off();

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
    _dance_update_flag = false;

    if((_taskPc >= 0) && (_taskPc < _taskProbBounds[0])){ // EXPLORE
      set_task(TASK_EXPLORE);
    }
    else if((_taskPc >= _taskProbBounds[0]) &&
            (_taskPc < _taskProbBounds[1])){ // REST
      set_task(TASK_REST);
    }
    else if((_taskPc >= _taskProbBounds[1]) &&
            (_taskPc < _taskProbBounds[2])){ // DANCE
      _dance_update_flag = true;
      set_task(TASK_DANCE);
    }
    else if((_taskPc >= _taskProbBounds[2]) &&
            (_taskPc < _taskProbBounds[3])){ // FINDHUMAN
      set_task(TASK_FINDHUMAN);
    }
    else if((_taskPc >= _taskProbBounds[3]) &&
            (_taskPc < _taskProbBounds[4])){ // FINDSOUND
      set_task(TASK_FINDSOUND);
    }
    else if((_taskPc >= _taskProbBounds[4]) &&
            (_taskPc < _taskProbBounds[5])){ // FINDLIGHT
      set_task(TASK_FINDLIGHT);
    }
    else if((_taskPc >= _taskProbBounds[5]) &&
            (_taskPc < _taskProbBounds[6])){ // FINDDARK
      set_task(TASK_FINDDARK);
    }
    else{ // EXPLORE
      set_task(TASK_EXPLORE);
    }

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
  ETaskCode _task_code = TASK_EXPLORE;
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
  bool _task_new_flag = true;

  // Sub task variables
  bool _dance_update_flag = false;
  uint32_t _dance_duration = 0;
  uint16_t _tantrum_duration = 0;

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
