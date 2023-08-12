//---------------------------------------------------------------------------
// PET BOT 3D - PB3D! 
// CLASS - TASK
//---------------------------------------------------------------------------
/*
The task class is part of the PetBot (PB) program. 

Author: Lloyd Fletcher
Date Created: 28th Aug. 2021
Date Edited:  28th Aug. 2021
*/

#ifndef TASK_H
#define TASK_H

#include <Arduino.h>
//#include <Adafruit_NeoPixel.h> // RGB LEDs
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include "Mood.h"
#include "Timer.h"

// Define task codes
#define TASK_TEST -7
#define TASK_PAUSE -4     // Only called by other tasks 
#define TASK_PICKEDUP -3  // Only called by other tasks 
#define TASK_INTERACT -2  // Only called by other tasks 
#define TASK_TANTRUM -1   // Only called by other tasks
#define TASK_EXPLORE 0    
#define TASK_REST 1       
#define TASK_DANCE 2      
#define TASK_FINDHUMAN 3
#define TASK_FINDSOUND 4
#define TASK_FINDLIGHT 5
#define TASK_FINDDARK 6 
#define TASK_POUNCE 7

class Task{
public:
  //---------------------------------------------------------------------------
  // CONSTRUCTOR - pass in pointers to main objects and other sensors
  Task(Adafruit_NeoPixel_ZeroDMA* RGBLEDs){
    _taskLEDs = RGBLEDs;
  }
  
  //---------------------------------------------------------------------------
  // BEGIN - called during setup function before main loop
  void begin(){
    // Generate a probability, start the task timer and set the task
    _taskPc = random(0,100); // NOTE: random num between (min,max-1)
    _taskTimer.start(_taskDuration);
    _LEDTimer.start(0);
  }

  //---------------------------------------------------------------------------
  // UPDATE - called during every iteration of the main loop
  void update(){
    if(_taskTimer.finished()){
      _update();
    }
  }

  void forceUpdate(){
    _update();
  }

  //---------------------------------------------------------------------------
  // GET,SET,RESET
  // Default setTask function uses default task durations
  void setTask(int8_t taskIn){
    if(taskIn != _taskCode){
      _taskCode = taskIn;
      _taskNewFlag = true;
      uint32_t tempDuration = random(_taskDurationMin,_taskDurationMax);
      
      // Set default duraiton for each task and set the LEDs
      if(_taskCode == TASK_TEST){
        _taskDuration = 30000;
        taskLEDTest();  
      }
      else if(_taskCode == TASK_EXPLORE){
        _taskDuration = tempDuration;
        taskLEDExplore();    
      }
      else if(_taskCode == TASK_REST){
        _taskDuration = tempDuration;  
      }
      else if(_taskCode == TASK_DANCE){
        _taskDuration = _danceDuration;  
        taskLEDDance();
      }
      else if(_taskCode == TASK_TANTRUM){
        _taskDuration = _tantrumDuration;
        taskLEDCollision();  
      }
      else if(_taskCode == TASK_FINDHUMAN){
        _taskDuration = 20000;
        taskLEDFindHuman();  
      }
      else if(_taskCode == TASK_FINDLIGHT){
        _taskDuration = 20000;
        taskLEDFindLight();  
      }
      else if(_taskCode == TASK_FINDDARK){
        _taskDuration = 20000;
        taskLEDFindDark();  
      }
      else if(_taskCode == TASK_FINDSOUND){
        _taskDuration = 20000;
        taskLEDFindSound();  
      }
      else if(_taskCode == TASK_INTERACT){
        _taskDuration = 20000;
        taskLEDInteract();  
      }      
      else if(_taskCode == TASK_PICKEDUP){
        _taskDuration = 60000;
        // LEDs set based on state in the TaskPickedUp class
      }
      else if(_taskCode == TASK_PAUSE){
        _taskDuration = 60000;
        // LEDs set based on state in the TaskPause class
      }
      else if(_taskCode == TASK_POUNCE){
        _taskDuration = 20000;
      }
      else{
        _taskCode = TASK_EXPLORE;
        _taskDuration = tempDuration;
        taskLEDExplore();
      }
      // Start the task timer
      _taskTimer.start(_taskDuration);
    }
  }

  void setTaskDuration(uint32_t taskDur){
    _taskDuration = taskDur;
    _taskTimer.start(_taskDuration);
  }

  int8_t getTask(){
    return _taskCode;
  }

  bool getNewTaskFlag(){
    return _taskNewFlag;
  }

  void setNewTaskFlag(bool inFlag){
    _taskNewFlag = inFlag;
  }

  void setDanceDuration(uint32_t inDuration){
    _danceDuration = inDuration;
  }

  void setTantrumDuration(uint16_t inDuration){
    _tantrumDuration = inDuration;
  }

  void setDanceUpdateFlag(bool inFlag){
    _danceUpdateFlag = inFlag;
  }

  bool getDanceUpdateFlag(){
    return _danceUpdateFlag;
  }

  void assignProb(int8_t moodIn){
     // Moods: [neutral,happy,sad,angry,scared]
     // test is used for forcing task probability
    if(moodIn == MOOD_NEUTRAL){
      _setTaskProb(_taskProbNeutral);  
    }
    else if(moodIn == MOOD_HAPPY){
      _setTaskProb(_taskProbHappy); 
    }
    else if(moodIn == MOOD_SAD){
      _setTaskProb(_taskProbSad); 
    }
    else if(moodIn == MOOD_ANGRY){
      _setTaskProb(_taskProbAngry);
    }
    else if(moodIn == MOOD_SCARED){
      _setTaskProb(_taskProbScared);
    }
    else if(moodIn == MOOD_TEST){
      _setTaskProb(_taskProbTest);
    }
    else{
      _setTaskProb(_taskProbNeutral);
    }

    // DEBUG
    /*
    Serial.println();
    Serial.print("TASK PROB BOUNDS: ");
    for(int8_t ii = 0; ii < _taskCount; ii++){
      Serial.print(_taskProbBounds[ii]), Serial.print(",");
    }
    Serial.println();
    */
  }

  //------------------------------------------------------------------------
  // TASK LED FUNCTIONS
  void taskLEDCollision(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 0));
    _taskLEDs->show();    
  }
  
  void taskLEDExplore(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 255, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 255, 0));
    _taskLEDs->show();    
  }
  
  void taskLEDRest(uint8_t intensity){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 0, intensity));
    _taskLEDs->setPixelColor(1, _taskLEDs->Color(0, 0, intensity));
    _taskLEDs->setPixelColor(2, _taskLEDs->Color(0, 0, intensity));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 0, intensity));
    _taskLEDs->show();
  }
  
  void taskLEDDance(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 255, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 255, 0));
    _taskLEDs->show();    
  }

  void taskLEDTantrum(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 0));
    _taskLEDs->show();     
  }
  
  void taskLEDFindHuman(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 255));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 255));
    _taskLEDs->show();    
  }
  
  void taskLEDInteract(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 255, 255));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 255, 255));
    _taskLEDs->show();    
  }

  void taskLEDPickedUpOk(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 255, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 255, 0));
    _taskLEDs->show();    
  }

  void taskLEDPickedUpPanic(){
    if(_LEDTimer.finished()){
      _LEDTimer.start(_LEDOnOffTime);
      _LEDSwitch = !_LEDSwitch;
    }
    
    if(_LEDSwitch){    
      _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 0));
      _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 0));
      _taskLEDs->show();
    }
    else{
      taskLEDOff();      
    }
  }

  void taskLEDPause(uint16_t pauseTime){
    if(_LEDTimer.finished()){
      _LEDTimer.start(pauseTime);
    }

    uint8_t LEDVal = _calcRisingLEDVal(pauseTime);
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(LEDVal, LEDVal, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(LEDVal, LEDVal, 0));
    _taskLEDs->show(); 
  }

  void taskLEDFindLight(){
    uint8_t intens = 255;
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(intens, intens, intens));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(intens, intens, intens));
    _taskLEDs->show();    
  }

  void taskLEDFindDark(){
    uint8_t intens = 85;
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(intens, intens, intens));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(intens, intens, intens));
    _taskLEDs->show();    
  }

  void taskLEDFindSound(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 0, 255));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 0, 255));
    _taskLEDs->show();    
  }

  void taskLEDTest(){
    if(_LEDTimer.finished()){
      _LEDTimer.start(_LEDOnOffTime);
      _LEDSwitch = !_LEDSwitch;
    }
    
    if(_LEDSwitch){  
      _taskLEDs->setPixelColor(0, _taskLEDs->Color(255, 0, 0));
      _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 0, 255));
      _taskLEDs->show();
    }
    else{
      _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 0, 255));
      _taskLEDs->setPixelColor(3, _taskLEDs->Color(255, 0, 0));
      _taskLEDs->show();  
    }
  }

  void taskLEDOff(){
    _taskLEDs->setPixelColor(0, _taskLEDs->Color(0, 0, 0));
    _taskLEDs->setPixelColor(3, _taskLEDs->Color(0, 0, 0));
    _taskLEDs->show();  
  }

  //------------------------------------------------------------------------
  // HSV LEDS
  void taskLEDHue(uint16_t hue){
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue)));
    _taskLEDs->show();   
  }

  void taskLEDHSV(uint16_t hue, uint8_t sat, uint8_t value){
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue,sat,value)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue,sat,value)));
    _taskLEDs->show();     
  }

  void taskLEDCol(uint16_t col){
    uint16_t hue = 65536*col/12;
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue)));
    _taskLEDs->show();    
  }

  void taskLEDCol(uint16_t colL, uint16_t colR){
    uint16_t hueL = 65536*colL/12;
    uint16_t hueR = 65536*colR/12;
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hueR)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hueL)));
    _taskLEDs->show();     
  }

  void taskLEDCSV(uint16_t col, uint8_t sat, uint8_t val){
    uint16_t hue = 65536*col/12;
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue,sat,val)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hue,sat,val)));
    _taskLEDs->show();     
  }

  void taskLEDCSV(uint16_t colL,uint16_t colR,uint8_t satL,uint8_t satR, uint8_t valL, uint8_t valR){
    uint16_t hueL = 65536*colL/12;
    uint16_t hueR = 65536*colR/12;
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hueR,satR,valR)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(hueL,satL,valL)));
    _taskLEDs->show();     
  }


  
  /*
  void taskLEDPounce(){
    taskLEDHue(_huePounce);
  }

  void taskLEDPounce(uint8_t intens){
    taskLEDHSV(_huePounce,255,intens);       
  }

  void taskLEDPounce(uint8_t intensL, uint8_t intensR){
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(_huePounce,255,intensR)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(_huePounce,255,intensL)));
    _taskLEDs->show();          
  }

  void taskLEDPounce(uint8_t satL, uint8_t satR, uint8_t intensL, uint8_t intensR){
    _taskLEDs->setPixelColor(0, _taskLEDs->gamma32(_taskLEDs->ColorHSV(_huePounce,satR,intensR)));
    _taskLEDs->setPixelColor(3, _taskLEDs->gamma32(_taskLEDs->ColorHSV(_huePounce,satL,intensL)));
    _taskLEDs->show();          
  }
  */

  //------------------------------------------------------------------------
  
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
    return round(float(startVal) + slope*float(_LEDTimer.getTime()));
  }

  uint8_t _calcRisingLEDVal(uint16_t timeInt){
    float startVal = 0.0,  endVal = 255.0;
    float slope = (float(startVal)-float(endVal))/(float(0.0)-float(timeInt));
    return round(float(startVal) + slope*float(_LEDTimer.getTime()));
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
