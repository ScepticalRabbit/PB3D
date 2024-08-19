#line 1 "/home/lloydf/Arduino/PB3D/PB3D-core/src/Speaker.h"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#ifndef SPEAKER_H
#define SPEAKER_H

#include <Arduino.h>
#include "PB3DConstants.h"
#include "Timer.h"

#define SPEAKER_POUT A0


class Speaker{
public:
  Speaker(){}

  //----------------------------------------------------------------------------
  // BEGIN: called once during SETUP
  void begin(){
    // NOTE: see adafruit guidance about metro M4 dac here:
    // https://learn.adafruit.com/adafruit-metro-m4-express-featuring-atsamd51/adapting-sketches-to-m0
    // pinMode(SPEAKER_POUT,OUTPUT);

    digitalWrite(SPEAKER_POUT,LOW);
    _soundTimer.start(0);
    _noteTimer.start(0);
    _songTimer.start(0);
  }

  void off(){
    noTone(SPEAKER_POUT);
    digitalWrite(SPEAKER_POUT,LOW);
  }

  //---------------------------------------------------------------------------
  // UPDATE: called during every LOOP
  void update(){
    // If the current sound has ended increment to play the next one
    if(_soundCount == SOUND_END){
      _soundStart = true;
      _soundCount = SOUND_PLAY;
      _soundCodeInd++;

      // If we have run out of codes in the array wait until reset is called
      // Reset must be called externally to this class
      if(_soundCodeInd >= _numCodes){
        _soundCount = SOUND_END;
        _soundCodeInd = _numCodes-1;
      }
    }

    if(_soundTimer.finished()){
      _soundTimer.start(_soundUpdateTime);

      uint8_t ss = _soundCodeInd;
      uint8_t ff = 2*_soundCodeInd;

      _currSoundCode = _soundCodes[ss];
      if(_currSoundCode == SPEAKER_SNORE){
        _soundDurs[ff] = _snoreDur;
      }
      else if (_currSoundCode == SPEAKER_GROWL){
        _soundDurs[ff] = _growlDur;
      }

      makeSound(_currSoundCode,
                _soundFreqs[ff],_soundFreqs[ff+1],
                _soundDurs[ff] ,_soundDurs[ff+1]);
    }
  }

  //----------------------------------------------------------------------------
  // SETTERS
  void setSoundCodes(uint8_t inCodes[], uint8_t numCodes){
    for(uint8_t ii=0; ii<numCodes; ii++){
      _soundCodes[ii] = inCodes[ii];
    }
  }

  void setSoundFreqs(uint16_t inFreqs[], uint8_t numFreqs){
    for(uint8_t ii=0; ii<numFreqs; ii++){
      _soundFreqs[ii] = inFreqs[ii];
    }
  }

  void setSoundDurs(uint16_t inDurs[], uint8_t numDurs){
    for(uint8_t ii=0; ii<numDurs; ii++){
      _soundDurs[ii] = inDurs[ii];
    }
  }

  //----------------------------------------------------------------------------
  // GETTERS
  uint8_t getCurrSoundCode(){return _currSoundCode;}
  uint8_t getSoundCount(){return _soundCount;}
  uint8_t getNumCodes(){return _numCodes;}
  uint8_t getNumFreqs(){return _numFreqs;}
  uint8_t getNumDurs(){return _numDurs;}

  uint32_t getTotalSoundDur(){
    uint32_t durSum = 0;
    for(uint8_t ii=0; ii<_numDurs; ii++){
      durSum += _soundDurs[ii];
    }
    return durSum;
  }

  //----------------------------------------------------------------------------
  // MAIN SOUND - ON
  void makeSound(uint8_t inCode, uint16_t freq1, uint16_t freq2, uint16_t onDur, uint16_t offDur){
    if(_soundStart){
      _soundStart = false;
      _soundCount = SOUND_PLAY;
      _noteTimer.start(onDur);
    }

    if(_noteTimer.finished()){
      _soundCount++;
      if(_soundCount == SOUND_PAUSE){
        _noteTimer.start(offDur);
      }
      else if(_soundCount >= SOUND_END){
        _soundCount = SOUND_END;
      }
    }

    if(_soundCount == SOUND_PLAY){
      if(inCode == SPEAKER_BEEP){
        beep(freq1);
      }
      else if(inCode == SPEAKER_SLIDE){
        slide(freq1,freq2,onDur);
      }
      else if(inCode == SPEAKER_GROWL){
        growl();
      }
      else if(inCode == SPEAKER_SCREECH){
        screech(freq1,freq2);
      }
      else if(inCode == SPEAKER_SNORE){
        snore();
      }
      else{
        off();
      }
    }
    else{
      off();
    }
  }

  //--------------------------------------------------------------------------------------------
  // MAIN SOUND - RESET
   void reset(){
    //Serial.println(F("SPEAKER: ext reset."));
    _soundStart = true;
    _soundCount = SOUND_PLAY;
    _soundCodeInd = 0;
  }

  //--------------------------------------------------------------------------------------------
  // SUB-SOUND - BEEP
  void beep(uint16_t beepFreq){
      tone(SPEAKER_POUT,beepFreq);
  }

  //--------------------------------------------------------------------------------------------
  // SUB-SOUND - SLIDE
   void slide(uint16_t startFreq, uint16_t endFreq, uint16_t onDur){
    uint16_t currFreq = _calcSlideFreq(startFreq,endFreq,onDur,_noteTimer.get_time());
    tone(SPEAKER_POUT,currFreq);
  }

  //--------------------------------------------------------------------------------------------
  // SUB-SOUND - GROWL
   void growl(){
    _growlOscillator = !_growlOscillator;
    if(_growlOscillator){
      tone(SPEAKER_POUT,_growlFreq1);
    }
    else{
      tone(SPEAKER_POUT,_growlFreq2);
    }
  }

  //----------------------------------------------------------------------------
  // SUB-SOUND - RANDOM SCREECH
  void screech(uint16_t freq1, uint16_t freq2){
    if(_randTimer.finished()){
      _randTimer.start(_screechRandInt);
      uint16_t currFreq = _genRandomFreq(freq1,freq2);
      tone(SPEAKER_POUT,currFreq);
    }
  }

  //----------------------------------------------------------------------------
  // SUB-SOUND - SNORE
   void snore(){
    _snoreCurrFreq = round(float(_snoreStartFreq) + _snoreSlope*float(_noteTimer.get_time()));
    tone(SPEAKER_POUT,_snoreCurrFreq);
  }

private:
  uint16_t _calcSlideFreq(uint16_t startNote, uint16_t endNote, uint16_t dur, uint16_t currTime){
    float slideSlope = (float(startNote)-float(endNote))/float(0.0-dur);
    uint16_t currFreq = round(float(startNote) + slideSlope*float(currTime));
    return currFreq;
  }

  uint16_t _genRandomFreq(uint16_t freq1, uint16_t freq2){
    uint16_t randFreq = 0;
    if(freq1 > freq2){
      randFreq = random(freq2,freq1);
    }
    else if(freq2 > freq1){
      randFreq = random(freq1,freq2);
    }
    return randFreq;
  }


  uint8_t _currSoundCode = SPEAKER_OFF;
  bool _soundStart = false;
  uint16_t _soundUpdateTime = 20;

  // FREQUENCIES AND TIMES
  uint8_t _soundCodeInd = 0;
  uint8_t _soundCount = 0;

  const static uint8_t _numCodes = 4;
  const static uint8_t _numFreqs = 8;
  const static uint8_t _numDurs = 8;
  uint8_t _soundCodes[_numCodes] = {SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
  uint16_t _soundFreqs[_numFreqs] = {NOTE_C5,NOTE_C5,NOTE_C5,NOTE_C5,NOTE_C5,NOTE_C5,NOTE_C5};
  uint16_t _soundDurs[_numDurs] = {100,100,100,100,100,100,100,100};

  // TIMERS
  Timer _soundTimer = Timer();
  Timer _songTimer = Timer();
  Timer _noteTimer = Timer();

  // BEEP BEEP Variables
  /*
  bool _bbStart = true;
  uint16_t _bbNoteDur = 100;
  uint16_t _bbPause = 100;
  uint8_t _bbNumBB = 2;
  uint8_t _bbNoteCount = 0;
  uint8_t _bbCount = 0;
  */

  // SLIDE Variables
  bool _startSlide = true;
  uint8_t _slideRepCount = 0;
  uint8_t _slideNoteCount = 0;

  // GROWL Variables
  bool _startGrowl = true;
  bool _growlOscillator = true;
  uint16_t _growlDur = 1000;
  uint16_t _growlFreq1 = NOTE_B2;
  uint16_t _growlFreq2 = NOTE_B3;

  // SCREECH Variables
  bool _startScreech = true;
  uint16_t _screechRandInt = 25;
  Timer _randTimer = Timer();

  // SNORE Variables
  bool _startSnore = true;
  uint16_t _snoreDur = 2000;
  uint16_t _snoreStartFreq = NOTE_C4;
  uint16_t _snoreEndFreq = NOTE_B2;
  float _snoreSlope = -float(_snoreStartFreq -_snoreEndFreq)/float(_snoreDur);
  uint16_t _snoreCurrFreq = _snoreStartFreq;
};
#endif // SPEAKER_H
