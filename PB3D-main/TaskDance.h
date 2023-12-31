//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS - TASKDANCE
//---------------------------------------------------------------------------
/*
The task ? class is part of the PetBot (PB) program. It is used to...

Author: Lloyd Fletcher
Date Created: 29th Aug 2021
Date Edited:  2nd Oct 2021 - PB3D
*/

#ifndef TASKDANCE_H
#define TASKDANCE_H

#include <Arduino.h>
#include "Mood.h"
#include "Task.h"
#include "Move.h"
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
  TaskDance(Mood* inMood, Task* inTask, Move* inMove, Speaker* inSpeaker){
    _moodObj = inMood;
    _taskObj = inTask;
    _moveObj = inMove;
    _speakerObj = inSpeaker;
  }

  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    _danceStartFlag = true;
    _timerObj.start(0);    
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every loop
  void update(){
    // Get new task flag reset after first main loop update
    if(_taskObj->getNewTaskFlag()){
      // Reset the dance start flag in case this is the current task.
      _danceStartFlag = true;
      
      if(_taskObj->getTask() == TASK_DANCE){
        _generateTempo();
        _generateDance();
        _taskObj->setDanceDuration(_danceDuration);
        _taskObj->setTaskDuration(_danceDuration);  
      }
    }
  }

  //---------------------------------------------------------------------------
  // DANCE
  void dance(){
    // At the start of the dance set and reset key variables
    if(_danceStartFlag){
      _startDance();
    }

    // Set the task LEDs on every loop
    _taskObj->taskLEDDance();

    // If needed set the speaker flags on every loop
    if(_speakerFlag){
      uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
      _speakerObj->setSoundCodes(inCodes,4);
    }
  
    // Update the dance move at given time interval, should be 1/4 note
    if(_timerObj.finished()){
      _updateDance();
    }
    
    // MAIN DANCE DECISION TREE
    if(_danceCurrMove == DANCE_STOP){
      _moveObj->stop();
    }
    else if(_danceCurrMove == DANCE_FORBACK){
      //Serial.println("DANCE_FORBACK");
      _moveObj->forwardBack(int(_dance4NoteMs),int(_dance4NoteMs));  
    }
    else if(_danceCurrMove == DANCE_CIRCLE){
      //Serial.println("DANCE_CIRCLE");
      _moveObj->circle();
    }
    else if(_danceCurrMove == DANCE_TURN){
      //Serial.println("DANCE_TURN");
      if(_danceTurnDir == MOVE_B_RIGHT){
        //Serial.println("DANCE TURN RIGHT.");
        _moveObj->turnToAngleCtrlSpd(90.0);
      }
      else{
        //Serial.println("DANCE TURN LEFT.");
        _moveObj->turnToAngleCtrlSpd(-90.0);
      }
    }
    else if(_danceCurrMove == DANCE_SPIN){
      //Serial.println("DANCE_SPIN");
     if(_danceSpinDir == MOVE_B_RIGHT){
        _moveObj->right();
      }
      else{
        _moveObj->left();
      }
    }
    else{
      //Serial.println("DANCE_WIGGLE");
      _moveObj->wiggle(int(_dance4NoteMs),int(_dance4NoteMs));
    }
  }


  //---------------------------------------------------------------------------
  // GET,SET,RESET
  float getDanceBarMs(){return _danceBarMs;}
  uint32_t getDuration(){return _danceDuration;}
  void setStartFlag(bool inFlag){_danceStartFlag = inFlag;}
  bool getStartFlag(){return _danceStartFlag;}
  bool getSpeakerFlag(){return _speakerFlag;}
  void setSpeakerFlag(bool inFlag){_speakerFlag = inFlag;}

private:
  //---------------------------------------------------------------------------
  // PRIVATE FUNCTIONS
  void _startDance(){
    _danceStartFlag = false;
    // Set initial variables
    _timerObj.start(round(_danceBarMs));
    _danceCurrBar = 0;
    _danceMoveInd = 0;
    _danceCurrMove = _danceMoveVec[_danceMoveInd];
    _moveObj->resetSubMoveTimer();

    // Increase mood when dance starts
    _moodObj->incMoodScore();

    // Set speaker
    if(_speakerFlag){
      _speakerObj->reset();
      uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
      _speakerObj->setSoundCodes(inCodes,4);
      uint16_t inFreqs[]  = {NOTE_G4,NOTE_G7,NOTE_G5,NOTE_G7,0,0,0,0};
      uint16_t inDurs[]   = {200,150,300,150,0,0,0,0};
      _speakerObj->setSoundFreqs(inFreqs,8);
      _speakerObj->setSoundDurs(inDurs,8);
    }
  }

  void _generateTempo(){
    _danceBPM = float(random(_danceBPMMin,_danceBPMMax)); // NOTE: random num between (min,max-1)
    _dance4NoteMs = (60.0/_danceBPM)*1000.0;
    _danceBarMs = _dance4NoteMs*4.0;
    _danceDuration = uint32_t(_danceBarMs*float(_danceNumBars)); 
  }

  void _generateDance(){
    // Random move that recurs throughout the dance
    uint8_t recurMove = random(1,3);  // NOTE: random num between (min,max-1)
    // Bars on which the recurrent move occurs, 0=B1 and B3, 1=B2 and B4
    uint8_t recurBeat = random(0,2); // NOTE: random num between (min,max-1)
    // 1 = Full 8 bar dance, 2 = Repeat 4 bar dance
    uint8_t recurBars = random(1,3); // NOTE: random num between (min,max-1)

    recurMove = 1;
    recurBeat = 0;
    //recurBars = 2;
    
    for(uint8_t ii=0 ; ii < _danceNumMoves ; ii++){
      if((recurBars == 2)&&(ii >= (_danceNumMoves/recurBars))){
        _danceMoveVec[ii] = _danceMoveVec[ii-(_danceNumMoves/recurBars)];
      }
      else{
        // If this is the recurring move set it, else generate
        if((ii%2)==recurBeat){
          _danceMoveVec[ii] = recurMove;
        }
        else{
          _danceMoveVec[ii] = random(1,DANCE_NUM_MOVES);
        } 
      }
    }
  }

  void _updateDance(){
    _timerObj.start(round(_danceBarMs));
    
    // Update the dance move
    _danceMoveInd = _danceMoveInd+1;
    if(_danceMoveInd >= _danceNumMoves){
       _danceMoveInd = 0;
    }  
    _danceCurrMove = _danceMoveVec[_danceMoveInd];

    // Update the dance bar
    _danceCurrBar = _danceCurrBar+1;
    // If the dance is over reset the count
    if(_danceCurrBar >= _danceNumBars){
      _danceStartFlag = true;
      _danceCurrBar = 0;
    }

    // Update turn/spin direction
    if(_danceMoveInd == 0){
      _danceTurnDir = MOVE_B_LEFT;
      _danceSpinDir = MOVE_B_LEFT;  
    }
    else if(_danceMoveVec[_danceMoveInd-1] == DANCE_TURN){
      if(_danceTurnDir == MOVE_B_RIGHT){
        _danceTurnDir = MOVE_B_LEFT;
      }
      else{
        _danceTurnDir = MOVE_B_RIGHT;  
      }
    }
    else if(_danceMoveVec[_danceMoveInd-1] == DANCE_SPIN){
      if(_danceSpinDir = MOVE_B_RIGHT){
        _danceSpinDir = MOVE_B_LEFT;
      }
      else{
        _danceSpinDir = MOVE_B_RIGHT;  
      }
    }

    // Reset the speaker to sync with dance bars
    _speakerObj->reset(); 
  }
  
  //---------------------------------------------------------------------------
  // MAIN OBJECT POINTERS
  Mood* _moodObj;
  Task* _taskObj;
  Move* _moveObj;
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
