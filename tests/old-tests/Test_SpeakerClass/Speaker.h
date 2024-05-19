//---------------------------------------------------------------------------
// PET BOT - PB3D! 
// CLASS - SPEAKER
//---------------------------------------------------------------------------
/*
The task class is part of the PetBot (PB) program. It used to...

Author: Lloyd Fletcher
Date Created: 19th February 2021
Date Edited:  2nd October 2021 - Started integration with PB3D 
*/

#ifndef SPEAKER_H
#define SPEAKER_H

#include "Timer.h"
#include "Notes.h"

#define SPEAKER_POUT A0

#define SPEAKER_OFF 0
#define SPEAKER_BEEPBEEP 1
#define SPEAKER_SLIDE 2
#define SPEAKER_SNORE 3
#define SPEAKER_GROWL 4
#define SPEAKER_SCREECH 5

class Speaker{
public:
  Speaker(){    
  }

  void begin(){
    pinMode(SPEAKER_POUT,OUTPUT);
    digitalWrite(SPEAKER_POUT,LOW);
    _soundTimer.start(0);
    _noteTimer.start(0);
    _songTimer.start(0);
  }

  void off(){
    noTone(SPEAKER_POUT);
    digitalWrite(SPEAKER_POUT,LOW);
  }

  void update(){
    if(_soundTimer.finished()){
      if(_soundCode == SPEAKER_BEEPBEEP){
        Serial.println("Update beep, beep");
        beepBeep2(1,_soundFreq1,_soundFreq2,_soundFreq3,_soundFreq4,
                    _soundDur1,_soundDur2,_soundDur3,_soundDur4); 
      }
      else if(_soundCode == SPEAKER_SLIDE){
        Serial.println("Update slide");
        slide2(1,_soundDur1,_soundFreq1,_soundFreq2,_soundDur2,
                 _soundDur3,_soundFreq3,_soundFreq4,_soundDur4); 
      }
      else if(_soundCode == SPEAKER_SNORE){
        Serial.println("Update snore");
        snore();  
      }
      else if(_soundCode == SPEAKER_GROWL){
        Serial.println("Update growl");
        growl(_soundDur1);  
      }
      else if(_soundCode == SPEAKER_SCREECH){
        Serial.println("Update screech");
        screech(_soundDur1);  
      }
      else{
        Serial.println("Off");
        off();
      }
      _soundTimer.start(_soundUpdateTime);
    }
  }

  void setSoundCode(uint8_t inCode){
    _soundCode = inCode;
  }

  void setSoundParams(uint8_t inCode,
                      uint16_t note1, uint16_t note2, uint16_t note3, uint16_t note4,
                      uint16_t dur1, uint16_t dur2, uint16_t dur3, uint16_t dur4){
    _soundCode = inCode;
    _soundFreq1 = note1;
    _soundFreq2 = note2;
    _soundFreq3 = note3;
    _soundFreq4 = note4;
    _soundDur1 = dur1;
    _soundDur2 = dur2;
    _soundDur3 = dur3;
    _soundDur4 = dur4;
  }

  void setSoundParams(uint16_t note1, uint16_t note2, uint16_t note3, uint16_t note4,
                      uint16_t dur1, uint16_t dur2, uint16_t dur3, uint16_t dur4){
    _soundFreq1 = note1;
    _soundFreq2 = note2;
    _soundFreq3 = note3;
    _soundFreq4 = note4;
    _soundDur1 = dur1;
    _soundDur2 = dur2;
    _soundDur3 = dur3;
    _soundDur4 = dur4;
  }

  void setSoundFreqs(uint16_t note1, uint16_t note2, uint16_t note3, uint16_t note4){
    _soundFreq1 = note1;
    _soundFreq2 = note2;
    _soundFreq3 = note3;
    _soundFreq4 = note4;
  }

  void setSoundDurs(uint16_t dur1, uint16_t dur2, uint16_t dur3, uint16_t dur4){
    _soundDur1 = dur1;
    _soundDur2 = dur2;
    _soundDur3 = dur3;
    _soundDur4 = dur4;
  }

  //--------------------------------------------------------------------------------------------
  // SOUND - BEEP BEEP
  //--------------------------------------------------------------------------------------------
  void resetBeepBeep(){
    _soundStart = true;
    _bbStart = true;
    _bbNoteCount = 0;
    _bbCount = 0;
  }

  uint8_t getBeepBeepCount(){
    return _bbCount;
  }

  void beepBeep2(uint8_t numBBs, uint16_t note1, uint16_t note2, uint16_t note3, uint16_t note4){
    // At beep, beep start set timers going and reset count
    if(_bbStart){
      _bbStart = false;
      _noteTimer.start(_bbNoteDur);
    }   

    if(_bbCount < numBBs){
      if(_noteTimer.finished()){
        
        // If current note is over increment counter
        _bbNoteCount++;
        if((_bbNoteCount%2) == 0){
          // If the count is even or 0 this is a note
          _noteTimer.start(_bbNoteDur);
        }
        else{
          // If the count is odd this is a pause
          _noteTimer.start(_bbPause);
        }
      }
      
      if(_bbNoteCount == 0){
        // 0 count is the first note
        tone(SPEAKER_POUT,note1,_bbNoteDur);
      }
      else if(_bbNoteCount == 2){
        // 2 count is the second note
        tone(SPEAKER_POUT,note2,_bbNoteDur);
      }
      else if(_bbNoteCount == 4){
        // 4 count is the second note
        tone(SPEAKER_POUT,note3,_bbNoteDur);
      }
      else if(_bbNoteCount == 6){
        // 2 count is the second note
        tone(SPEAKER_POUT,note4,_bbNoteDur);
      }
      else if(_bbNoteCount >= 8){
        _bbCount++;
        _bbNoteCount = 0;
        _noteTimer.start(_bbNoteDur);
      }
      else{
        // Everything else is a pause
        off();
      }
    }
  }

  void beepBeep2(uint8_t numBBs, 
                 uint16_t note1, uint16_t note2, uint16_t note3, uint16_t note4,
                 uint16_t n1Dur, uint16_t n2Dur, uint16_t n3Dur, uint16_t n4Dur){
    // At beep, beep start set timers going and reset count
    if(_bbStart){
      _bbStart = false;
      _noteTimer.start(n1Dur);
    }   

    if(_bbCount < numBBs){
      if(_noteTimer.finished()){
        // If current note is over increment counter
        _bbNoteCount++;
        if(_bbNoteCount == 0){
          _noteTimer.start(n1Dur);
        }
        else if(_bbNoteCount == 2){
          _noteTimer.start(n2Dur);
        }
        else if(_bbNoteCount == 4){
          _noteTimer.start(n3Dur);
        }
        else if(_bbNoteCount == 6){
          _noteTimer.start(n4Dur);
        }
        else{
          _noteTimer.start(_bbPause);
        }
      }
      
      if(_bbNoteCount == 0){
        // 0 count is the first note
        tone(SPEAKER_POUT,note1,n1Dur);
      }
      else if(_bbNoteCount == 2){
        // 2 count is the second note
        tone(SPEAKER_POUT,note2,n2Dur);
      }
      else if(_bbNoteCount == 4){
        // 4 count is the second note
        tone(SPEAKER_POUT,note3,n3Dur);
      }
      else if(_bbNoteCount == 6){
        // 2 count is the second note
        tone(SPEAKER_POUT,note4,n4Dur);
      }
      else if(_bbNoteCount >= 8){
        _bbCount++;
        _bbNoteCount = 0;
        _noteTimer.start(n1Dur);
      }
      else{
        // Everything else is a pause
        off();
      }
    }
  }
  
  //--------------------------------------------------------------------------------------------
  // SOUND - GROWL
  //--------------------------------------------------------------------------------------------
  void resetGrowl(){
    _soundStart = true;
    _startGrowl = true;
  }

  void growl(uint16_t growlDur){
    if(_startGrowl){
      _startGrowl = false;
      _noteTimer.start(growlDur);
    }
    
    if(_noteTimer.finished()){
      off();
    }
    else{
      _growlOscillator = !_growlOscillator;
      if(_growlOscillator){
        tone(SPEAKER_POUT,_growlFreq1,growlDur);
      }
      else
      {
        tone(SPEAKER_POUT,_growlFreq2,growlDur);
      }
    }
  }

  //--------------------------------------------------------------------------------------------
  // SOUND - SCREECH
  //--------------------------------------------------------------------------------------------
  void resetScreech(){
    _soundStart = true;
    _startScreech = true;
  }
  
  void screech(uint16_t screechDur){
    if(_startScreech){
      _startScreech = false;
      _noteTimer.start(screechDur);
    }
    
    if(_noteTimer.finished()){
      off();
    }
    else{
      uint16_t currFreq = random(_screechMinFreq,_screechMaxFreq);
      tone(SPEAKER_POUT,currFreq,screechDur);
    }
  }

  //--------------------------------------------------------------------------------------------
  // SOUND - SNORE
  //--------------------------------------------------------------------------------------------
  void resetSnore(){
    Serial.println("Reset snore function");
    _soundStart = true;
    _startSnore = true;
  }

  void snore(){
    if(_startSnore){
      Serial.println("Start snore");
      _startSnore = false;
      _noteTimer.start(_snoreDur);
    }
    
    if(_noteTimer.finished()){
      off();
    }
    else{
      _snoreCurrFreq = round(float(_snoreStartFreq) + _snoreSlope*float(_noteTimer.getTime()));
      tone(SPEAKER_POUT,_snoreCurrFreq,_snoreDur);
    }
  }

  //--------------------------------------------------------------------------------------------
  // SOUND - SLIDE
  //--------------------------------------------------------------------------------------------
  void resetSlide(){
    _soundStart = true;
    _startSlide = true;
    _slideRepCount = 0;
    _slideNoteCount = 0;
  }

  void slide2(uint8_t numReps, 
              uint16_t s1Dur, uint16_t s1StartNote, uint16_t s1EndNote, uint16_t s1Pause,
              uint16_t s2Dur, uint16_t s2StartNote, uint16_t s2EndNote, uint16_t s2Pause){
                
    if(_startSlide){
      _startSlide = false;
      _noteTimer.start(s1Dur);  
    }

    if(_slideRepCount < numReps){
      if(_noteTimer.finished()){
        // If current note is over increment counter
        _slideNoteCount++;
        if(_slideNoteCount == 0){
          _noteTimer.start(s1Dur);
        }
        else if(_slideNoteCount == 1){
          _noteTimer.start(s1Pause);  
        }
        else if(_slideNoteCount == 2){
          _noteTimer.start(s2Dur);
        }
        else if(_slideNoteCount == 3){
          _noteTimer.start(s2Pause);  
        }
        else{
          _noteTimer.start(s1Pause);
        }
      }
      
      if(_slideNoteCount == 0){
        // 0 count is the first slide
        float slideSlope = (float(s1StartNote)-float(s1EndNote))/float(0.0-s1Dur);
        uint16_t currFreq = round(float(s1StartNote) + slideSlope*float(_noteTimer.getTime()));
        tone(SPEAKER_POUT,currFreq,s1Dur);
      }
      else if(_slideNoteCount == 2){
        // 2 count is the second slide
        float slideSlope = (float(s2StartNote)-float(s2EndNote))/float(0.0-s2Dur);
        uint16_t currFreq = round(float(s2StartNote) + slideSlope*float(_noteTimer.getTime()));
        tone(SPEAKER_POUT,currFreq,s2Dur);
      }
      else if(_slideNoteCount >= 4){
        _slideRepCount++;
        _slideNoteCount = 0;
        _noteTimer.start(s1Dur);
      }
      else{
        // Everything else is a pause
        off();
      }
    }
  }

  //--------------------------------------------------------------------------------------------
  // SOUND - RESET
  //--------------------------------------------------------------------------------------------
  void reset(){
    _soundStart = true;
    resetSnore();
    resetGrowl();
    resetScreech();
    resetBeepBeep();
    resetSlide();
  }

private:
  //------------------------------------------------------------------------
  // Variables
  uint8_t _soundCode = SPEAKER_OFF;
  bool _soundStart = false;
  uint16_t _soundUpdateTime = 20; 
  uint16_t _soundFreq1 = NOTE_C5;
  uint16_t _soundFreq2 = NOTE_C5;
  uint16_t _soundFreq3 = NOTE_C5;
  uint16_t _soundFreq4 = NOTE_C5;
  uint16_t _soundDur1 = 100;
  uint16_t _soundDur2 = 100;
  uint16_t _soundDur3 = 100;
  uint16_t _soundDur4 = 100;
  
  // TIMERS
  Timer _soundTimer;
  Timer _songTimer;
  Timer _noteTimer;

  // BEEP BEEP Variables 
  bool _bbStart = true;
  uint16_t _bbNoteDur = 100;
  uint16_t _bbPause = 100;
  uint8_t _bbNumBB = 2;
  uint8_t _bbNoteCount = 0;
  uint8_t _bbCount = 0;

  // SLIDE Variables
  bool _startSlide = true;
  uint8_t _slideRepCount = 0;
  uint8_t _slideNoteCount = 0;

  // GROWL Variables
  bool _startGrowl = true;
  bool _growlOscillator = true;
  uint16_t _growlFreq1 = NOTE_B2;
  uint16_t _growlFreq2 = NOTE_B3; 

  // SCREECH Variables
  bool _startScreech = true;
  uint16_t _screechMinFreq = NOTE_B2;
  uint16_t _screechMaxFreq = NOTE_C8;

  // SNORE Variables
  bool _startSnore = true;
  uint16_t _snoreDur = 2000;
  uint16_t _snoreStartFreq = NOTE_C4; 
  uint16_t _snoreEndFreq = NOTE_B2;
  float _snoreSlope = -float(_snoreStartFreq -_snoreEndFreq)/float(_snoreDur);
  uint16_t _snoreCurrFreq = _snoreStartFreq;
};
#endif // SPEAKER_H
