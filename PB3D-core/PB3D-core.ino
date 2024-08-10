//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

// External Classes - Sensor and Module Libraries
#include <Adafruit_MotorShield.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>

// Internal Classes - Basic Functions
#include "src/CollisionManager.h"
#include "src/Encoder.h"
#include "src/FilterLowPass.h"
#include "src/FilterMovAvg.h"
#include "src/MoodManager.h"
#include "src/MoveManager.h"
#include "src/PatSensor.h"
#include "src/PID.h"
#include "src/Speaker.h"
#include "src/Tail.h"
#include "src/TaskManager.h"
#include "src/Timer.h"
#include "src/I2CDataSender.h"
#include "src/IMUSensor.h"
#include "src/Navigation.h"
// Internal Classes - Tasks
#include "src/TaskDance.h"
#include "src/TaskRest.h"
#include "src/TaskTantrum.h"
#include "src/TaskInteract.h"
#include "src/TaskFindHuman.h"
#include "src/TaskFindLight.h"
#include "src/TaskFindSound.h"
#include "src/TaskPickedUp.h"
#include "src/TaskPounce.h"
#include "src/TaskPause.h"

//-----------------------------------------------------------------------------
// VARIABLES
//-----------------------------------------------------------------------------

// Debugging - Codes
bool _debug_collisionOff = false;
bool _debug_forceMood = true;
int8_t _debug_moodCode = MOOD_NEUTRAL;
bool _debug_forceTask = true;
int8_t _debug_taskCode = TASK_EXPLORE;
bool _debug_forceMove = false;
int8_t _debug_moveType = MOVE_C_CIRCLE;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TEST VARIABLES
bool _test_switch = true;
uint16_t _test_time= 2000;
Timer _test_timer = Timer();

bool _test_retest = true;

float _test_value = 360.0; // Can also set to float
//uint8_t _test_value = 80;

uint16_t _test_reportTime = 200;
Timer _test_reportTimer = Timer();

bool _test_pauseSwitch = true;
uint8_t _test_count = 0;
uint8_t _test_countLim = 4;
uint32_t _test_pauseTime = 2000;
Timer _test_pauseTimer = Timer();

bool _test_firstLoop = true;

uint32_t _test_timeStamp = 0;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//-----------------------------------------------------------------------------
// EXTERNAL CLASSES

// MOOD LEDs
Adafruit_NeoPixel_ZeroDMA leds = Adafruit_NeoPixel_ZeroDMA(
  MOOD_NUMPIX, MOOD_PIN, NEO_GRB + NEO_KHZ800);

// MOTORS - Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//-----------------------------------------------------------------------------
// INTERNAL CLASSES

// SENSORS AND CONTROLLERS
static int encPinAL = 2, encPinBL = 3;
static int encPinAR = 4, encPinBR = 5;
// NOTE: encoders must be in main to attach interrupts
Encoder encoderL = Encoder(encPinAL,encPinBL);
Encoder encoderR = Encoder(encPinAR,encPinBR);

// BASIC CLASSES
MoveManager moveObj = MoveManager(&AFMS,&encoderL,&encoderR);
MoodManager moodObj = MoodManager(&leds);
TaskManager taskObj = TaskManager(&leds);
CollisionManager collisionObj = CollisionManager(&moodObj,&taskObj,&moveObj);

// Sensors, Actuators and Comms
Speaker speakerObj = Speaker();
PatSensor patSensorObj = PatSensor();
Tail tailObj = Tail();
IMUSensor IMUObj = IMUSensor();
Navigation navObj = Navigation(&encoderL,&encoderR,&IMUObj);
I2CDataSender senderObj = I2CDataSender(&collisionObj,&moodObj,&taskObj,&moveObj,&IMUObj,&navObj);

// TASK CLASSES - Uses basic classes
TaskDance taskDanceObj = TaskDance(&moodObj,&taskObj,&moveObj,&speakerObj);
TaskRest taskRestObj = TaskRest(&moodObj,&taskObj,&moveObj,&speakerObj);
TaskTantrum taskTantrumObj = TaskTantrum(&moodObj,&taskObj,&moveObj,&speakerObj);
TaskInteract taskInteractObj = TaskInteract(&moodObj,&taskObj,&moveObj,&speakerObj,&taskDanceObj,&patSensorObj);
TaskFindHuman taskFindHumanObj = TaskFindHuman(&moodObj,&taskObj,&moveObj,&speakerObj,&taskInteractObj);
TaskFindLight taskFindLightObj = TaskFindLight(&moodObj,&taskObj,&moveObj,&speakerObj,&patSensorObj);
TaskFindSound taskFindSoundObj = TaskFindSound(&moodObj,&taskObj,&moveObj,&speakerObj);
TaskPickedUp taskPickedUpObj = TaskPickedUp(&collisionObj,&moodObj,&taskObj,&moveObj,&speakerObj,&patSensorObj);
TaskPounce taskPounceObj = TaskPounce(&collisionObj,&moodObj,&taskObj,&moveObj,&speakerObj);
TaskPause taskPauseObj = TaskPause(&collisionObj,&taskObj,&moveObj,&speakerObj);

//-----------------------------------------------------------------------------
// SETUP
void setup() {
  // Start the serial
  Serial.begin(115200);
  // Only use below to stop start up until USB cable connected
  // while(!Serial){}

  // Initialize I2C communications for sensors and sub boards
  Wire.begin();  // Join I2C bus as leader
  delay(2000);   // Needed to ensure sensors work, delay allows sub-processors to start up

  // SERIAL: POST SETUP
  Serial.println();
  Serial.println(F("PB3D: SETUP"));
  Serial.println(F("------------------------------"));

  // Seed the random generator - used by many classes 'begin' function
  randomSeed(analogRead(A1));

  leds.begin();
  leds.show();

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // TEST CODE

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // NOTE: I2C multiplexer and light sensors must be initialised first!
  taskFindLightObj.begin();

  moodObj.begin();
  taskObj.begin();
  moveObj.begin();
  collisionObj.begin();

  speakerObj.begin();
  patSensorObj.begin();
  tailObj.begin();
  IMUObj.begin();
  navObj.begin();
  senderObj.begin();

  taskFindHumanObj.begin();
  taskInteractObj.begin();
  taskPickedUpObj.begin();
  taskPounceObj.begin();
  taskPauseObj.begin();

  // Make sure all the laser sensors are left on
  taskFindSoundObj.setSendByte(B00111110);
  taskFindSoundObj.begin();

  // Pass durations to the master task object
  taskObj.setDanceDuration(taskDanceObj.getDuration());
  taskObj.setTantrumDuration(taskTantrumObj.getDuration());

  // Start timers in the main program
  _test_timer.start(0);
  _test_reportTimer.start(0);
  _test_pauseTimer.start(_test_pauseTime);

  // Encoders - Attach Interrupt Pins - CHANGE,RISING,FALLING
  attachInterrupt(digitalPinToInterrupt(encPinAL),
                  updateEncLA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinAR),
                  updateEncRA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinBL),
                  updateEncLB,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinBR),
                  updateEncRB,CHANGE);

  //-------------------------------------------------------------------------
  // DEBUG MODE:
  if(_debug_forceMood){
    moodObj.setMood(_debug_moodCode);
  }
  if(_debug_forceTask){
    taskObj.setTask(_debug_taskCode);
  }
  if(_debug_forceMove){
    moveObj.updateMove(_debug_moveType);
  }
  taskObj.assignProb(moodObj.getMood());

  tailObj.setState(TAIL_CENT);
  Serial.println();

  // Final setup - increase I2C clock speed
  Wire.setClock(400000); // 400KHz

  //while(1){}
}

//------------------------------------------------------------------------------
// MAIN LOOP
void loop(){
  //if(!_test_firstLoop){while(true){};}
  //uint32_t startLoop = millis();
  uint32_t startLoop = micros();

  //----------------------------------------------------------------------------
  // UPDATE MOOD - Based on internal timer, see Mood class
  moodObj.update();

  // If mood has updated then modify the other classes
  if(moodObj.getNewMoodFlag()){
    moodObj.setNewMoodFlag(false); // Reset the flag
    taskObj.assignProb(moodObj.getMood());
    moveObj.setPWRByDiff(moodObj.getPowerDiff());
    moveObj.setSpeedByMoodFact(moodObj.getSpeedFact());
  }
  if(_debug_forceMood){moodObj.setMood(_debug_moodCode);}

  //----------------------------------------------------------------------------
  // GENERATE TASK - Based on internal timer, see Task class
  taskObj.update();

  // If task has changed update mood LEDs
  if(taskObj.getNewTaskFlag()){moodObj.resetMood();}

  // DEBUG MODE: force task to a particular value
  if(_debug_forceTask){
    if(taskObj.getTask() != _debug_taskCode){
      taskObj.setTask(_debug_taskCode);
    }
  }
  if(_debug_collisionOff){collisionObj.set_enabled_flag(false);}

  //----------------------------------------------------------------------------
  // UPDATE OBJECTS - Each object has own internal timer
  uint32_t startUpdate = micros();
  collisionObj.update();
  uint32_t endUpdate = micros();

  speakerObj.update();
  patSensorObj.update();
  tailObj.update();
  IMUObj.update();
  encoderL.updateSpeed();
  encoderR.updateSpeed();
  navObj.update();
  senderObj.update();

  taskDanceObj.update();
  taskFindHumanObj.update();
  taskFindLightObj.update();
  taskFindSoundObj.update();
  taskInteractObj.update();
  taskPauseObj.update();
  taskPickedUpObj.update();
  taskRestObj.update();

  // After updating all objects reset the new task flag
  taskObj.setNewTaskFlag(false);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // TEST CODE - SENSOR REPORTS
  if(_test_reportTimer.finished()){
    _test_reportTimer.start(_test_reportTime);
    //DEBUG_PrintColCheck();
    //DEBUG_PrintAllRanges();
    //DEBUG_PrintLightSens();
  }
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //----------------------------------------------------------------------------
  // RESET FLAGS
  // Reset the sound code and let the decision tree update it for the next loop
  uint8_t inCodes[] = {SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF,SPEAKER_OFF};
  speakerObj.setSoundCodes(inCodes,4);
  // Reset the tail to the central position - allow tasks and mood control
  tailObj.setState(TAIL_CENT);

  //----------------------------------------------------------------------------
  // MAIN DECISION TREE
  //
  // NOTE: Rest and Interact are placed before collision avoidance to disable
  // handling of collisions while these modes are active - should be able to
  // make this smarter so certain things can re-enable collision avoidance
  if(taskObj.getTask() == TASK_TEST){
    //DEBUG_SpeedTest(_test_value,MOVE_B_FORWARD);
  }
  else if(taskObj.getTask() == TASK_REST){
    taskRestObj.rest();
  }
  else if(taskObj.getTask() == TASK_PAUSE){
    taskPauseObj.pause();
  }
  else if(taskObj.getTask() == TASK_PICKEDUP){
    taskPickedUpObj.pickedUp();
  }
  else if(taskObj.getTask() == TASK_INTERACT){
    taskInteractObj.interact();
  }
  else if(collisionObj.getEscapeFlag() && !_debug_collisionOff){
    //Serial.println("======================ESCAPE======================");
    escapeCollision(); // SEE FUNCTION DEF BELOW MAIN
  }
  else if(collisionObj.getDetectFlag() && !_debug_collisionOff){
    //DEBUG_PrintColFlags();
    detectedCollision(); // SEE FUNCTION DEF BELOW MAIN
  }
  else if(taskObj.getTask() == TASK_DANCE){
    if(taskObj.getDanceUpdateFlag()){
      taskObj.setDanceUpdateFlag(false);
      taskDanceObj.setSpeakerFlag(false);
    }
    taskDanceObj.dance();
  }
  else if(taskObj.getTask() == TASK_FINDHUMAN){
    taskFindHumanObj.findHuman();
  }
  else if(taskObj.getTask() == TASK_FINDLIGHT){
    taskFindLightObj.findLight();
  }
  else if(taskObj.getTask() == TASK_FINDDARK){
    taskFindLightObj.findDark();
  }
  else if(taskObj.getTask() == TASK_FINDSOUND){
    taskFindSoundObj.findSound();
  }
  else if(taskObj.getTask() == TASK_POUNCE){
    taskPounceObj.seekAndPounce();
  }
  else if(taskObj.getTask() == TASK_TANTRUM){
    if(taskTantrumObj.getCompleteFlag()){
      taskObj.forceUpdate();
    }
    else{
      taskTantrumObj.haveTantrum();
    }
  }
  else{
    taskObj.taskLEDExplore();

    // MOVEMENT: Type Update
    if(_debug_forceMove){
      moveObj.updateMove(_debug_moveType);
    }
    else{
      moveObj.updateMove();
    }
    // MOVEMENT: Call current movement function
    moveObj.go();
  }

  //-------------------------------------------------------------------------
  // POST DECISION TREE OVERRIDES
  // Wag tail if happy - regardless of task use of tail
  if(moodObj.getMood() == MOOD_HAPPY){
    tailObj.setState(TAIL_WAG_INT);
    // wagmovetime,wagoffset,_test_pauseTime,wagcount
    tailObj.setWagParams(200,30,4000,6);
  }

  // If sleeping don't wag tail
  if(taskObj.getTask() == TASK_REST){
    tailObj.setState(TAIL_CENT);
  }

  //-------------------------------------------------------------------------
  // END of LOOP

  //uint32_t endLoop = millis();
  //Serial.print(F("MAIN LOOP TOOK: "));
  //Serial.print(endLoop-startLoop); Serial.print(",");
  //Serial.print(endUpdate-startUpdate); Serial.print(",");
  //Serial.println();
  //_test_firstLoop = false;

}

//------------------------------------------------------------------------------
// INTERRUPT FUNCTIONS
void updateEncLA(){
  encoderL.updateNEQ();
}
void updateEncLB(){
  encoderL.updateEQ();
}
void updateEncRA(){
  encoderR.updateEQ();
}
void updateEncRB(){
  encoderR.updateNEQ();
}

//------------------------------------------------------------------------------
// COLLISION HANDLING TASK TREE FUNCTIONS
void escapeCollision(){
  taskObj.taskLEDCollision();
  collisionObj.resetFlags();

  if(collisionObj.getBeepBeepFlag()){
    uint8_t inCodes[] = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    speakerObj.setSoundCodes(inCodes,4);
  }

  collisionObj.escape();
}

void detectedCollision(){
  // Turn on collision LEDs and set escape flags
  taskObj.taskLEDCollision();
  collisionObj.setEscapeStart();

  // If we are moving in a circle or a spiral then switch direction
  moveObj.changeCircDir();

  // Reset the PIDs and stop the motors
  moveObj.stop();

  // If the bumper flag was tripped we need to go beep,beep!
  collisionObj.setBeepBeepFlag(false);
  if(collisionObj.getBumperFlag()){
    collisionObj.setBeepBeepFlag(true);

    speakerObj.reset();
    uint8_t inCodes[]   = {SPEAKER_SLIDE,SPEAKER_SLIDE,SPEAKER_OFF,SPEAKER_OFF};
    speakerObj.setSoundCodes(inCodes,4);
    uint16_t inFreqs[]  = {NOTE_G4,NOTE_FS4,NOTE_G4,NOTE_FS4,0,0,0,0};
    speakerObj.setSoundFreqs(inFreqs,8);
    uint16_t inDurs[]   = {200,100,200,100,0,0,0,0};
    speakerObj.setSoundDurs(inDurs,8);
  }

  // Call specific tasks that need to handle collision events
  if((taskObj.getTask() == TASK_FINDLIGHT)||(taskObj.getTask() == TASK_FINDDARK)){
    taskFindLightObj.resetGrad();
  }
  if((taskObj.getTask() == TASK_POUNCE) && (taskPounceObj.getState() == POUNCE_RUN)){
    taskPounceObj.collisionResetToRealign();

    int16_t angCent = taskPounceObj.getAngCentForCollision();
    if(collisionObj.getEscapeTurn() == MOVE_B_LEFT){
      taskPounceObj.setRealignCent(-1*angCent);
    }
    else{
      taskPounceObj.setRealignCent(1*angCent);
    }
  }

  // Reset the collision flags
  collisionObj.resetFlags();
  collisionObj.incCount();
  if(collisionObj.getCount() >= taskTantrumObj.getThreshold()){
    collisionObj.resetCount();
    taskTantrumObj.setStartTantrumFlag();
    taskObj.setTask(TASK_TANTRUM);
  }
}

//------------------------------------------------------------------------------
// DEBUG FUNCTIONS
/*
void DEBUG_SpeedTest(uint8_t inPWR, uint8_t moveCode){
  if(_test_pauseTimer.finished()){
    if(_test_pauseSwitch){
      _test_pauseSwitch = false;
      _test_timer.start(_test_time);
      _test_switch = true;
    }
    DEBUG_PlotSpeedMMPS();

    if(_test_timer.finished()){
      if(_test_retest){
        _test_timer.start(_test_time);
        _test_switch = !_test_switch;
      }
      else{
        _test_switch = false;
      }
    }

    if(_test_switch){
      if(moveCode == MOVE_B_BACK){
        moveObj.backPWR(inPWR);
      }
      else if(moveCode == MOVE_B_LEFT){
        moveObj.leftPWR(inPWR);
      }
      else if(moveCode == MOVE_B_RIGHT){
        moveObj.rightPWR(inPWR);
      }
      else{
        moveObj.forwardPWR(inPWR);
      }
    }
    else{
      moveObj.stop();
    }
  }
  else{
    moveObj.stop();
  }
}

void DEBUG_SpeedTest(float inSpeed, uint8_t moveCode){
  if(_test_pauseTimer.finished()){
    if(_test_pauseSwitch){
      _test_pauseSwitch = false;
      _test_timer.start(_test_time);
      _test_switch = true;
    }
    DEBUG_PlotSpeedMMPS();

    if(_test_timer.finished()){
      if(_test_retest){
        _test_timer.start(_test_time);
        _test_switch = !_test_switch;
      }
      else{
        _test_switch = false;
      }
    }

    if(_test_switch){
      if(moveCode == MOVE_B_BACK){
        moveObj.back(inSpeed);
      }
      else if(moveCode == MOVE_B_LEFT){
        moveObj.left(inSpeed);
      }
      else if(moveCode == MOVE_B_RIGHT){
        moveObj.right(inSpeed);
      }
      else{
        moveObj.forward(inSpeed);
      }
    }
    else{
      moveObj.stop();
    }
  }
  else{
    moveObj.stop();
  }
}

void DEBUG_PrintSpeedMMPS(){
  Serial.println();
  Serial.print(F("ENCODER SPEED, L="));
  Serial.print(encoderL.getSmoothSpeedMMPS());
  Serial.print(F(" mm/s, R = "));
  Serial.print(encoderR.getSmoothSpeedMMPS());
  Serial.print(F(" mm/s"));
  Serial.println();
}

void DEBUG_PrintSpeedCPS(){
  Serial.println();
  Serial.print(F("ENCODER SPEED, L="));
  Serial.print(encoderL.getSmoothSpeedMMPS());
  Serial.print(F(" mm/s, R = "));
  Serial.print(encoderR.getSmoothSpeedMMPS());
  Serial.print(F(" mm/s"));
  Serial.println();
}

void DEBUG_PlotSpeedBoth(){
  Serial.print(encoderL.getSmoothSpeedCPS());
  Serial.print(",");
  Serial.print(encoderL.getSmoothSpeedMMPS());
  Serial.print(",");
  Serial.print(encoderR.getSmoothSpeedCPS());
  Serial.print(",");
  Serial.print(encoderR.getSmoothSpeedMMPS());
  Serial.print(",");
  Serial.println();
}

void DEBUG_PlotSpeedPID_L(){
  Serial.print(moveObj.getSpeedPIDSetPoint_L());
  Serial.print(",");
  Serial.print(moveObj.getSpeedPIDOutput_L());
  Serial.print(",");
  Serial.print(moveObj.getSpeedPIDP_L());
  Serial.print(",");
  Serial.print(moveObj.getSpeedPIDI_L());
  Serial.print(",");
  Serial.print(moveObj.getSpeedPIDD_L());
  Serial.print(",");
  Serial.print(encoderL.getSmoothSpeedMMPS());
  Serial.print(",");
  Serial.println();
}

void DEBUG_PlotSpeedPID_R(){
  Serial.print(moveObj.getSpeedPIDSetPoint_R());
  Serial.print(",");
  Serial.print(moveObj.getSpeedPIDOutput_R());
  Serial.print(",");
  Serial.print(moveObj.getSpeedPIDP_R());
  Serial.print(",");
  Serial.print(moveObj.getSpeedPIDI_R());
  Serial.print(",");
  Serial.print(moveObj.getSpeedPIDD_R());
  Serial.print(",");
  Serial.print(encoderR.getSmoothSpeedMMPS());
  Serial.print(",");
  Serial.println();
}

void DEBUG_PlotSpeedMMPS(){
  uint32_t thisTime = millis();
  Serial.print(thisTime-_test_timeStamp);
  _test_timeStamp = thisTime;
  Serial.print(",");
  Serial.print(encoderL.getSmoothSpeedMMPS());
  Serial.print(",");
  Serial.print(encoderR.getSmoothSpeedMMPS());
  Serial.print(",");
  Serial.println();
}

void DEBUG_PrintColCheck(){
  Serial.println();
  Serial.println(F("------------------------------"));
  Serial.println("CheckVec=[BL,BR,US,LL,LR,LU,LD,]");
  Serial.print("CheckVec=[");
  for(uint8_t ii=0;ii<7;ii++){
      Serial.print(" ");Serial.print(collisionObj.getColCheck(ii));Serial.print(",");
  }
  Serial.println("]");

  Serial.print(F("US: "));
  //Serial.print(collisionObj.getColUSFlag());
  Serial.print(F(" , "));
  Serial.print(collisionObj.getUSRangeMM());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,L: "));
  //Serial.print(collisionObj.getColLSRFlagL());
  Serial.print(F(" , "));
  Serial.print(collisionObj.getLSRRangeL());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,R: "));
  //Serial.print(collisionObj.getColLSRFlagR());
  Serial.print(F(" , "));
  Serial.print(collisionObj.getLSRRangeR());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,A: "));
  //Serial.print(collisionObj.getColLSRFlagB());
  Serial.print(F(" , "));
  Serial.print(collisionObj.getLSRRangeA());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,U: "));
  //Serial.print(collisionObj.getColLSRFlagU());
  Serial.print(F(" , "));
  Serial.print(collisionObj.getLSRRangeU());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,D: "));
  //Serial.print(collisionObj.getColLSRFlagD());
  Serial.print(F(" , "));
  Serial.print(collisionObj.getLSRRangeD());
  Serial.println(F(" mm"));

  Serial.println(F("------------------------------"));
  Serial.println();
}//---------------------------------------------------------------------------

void DEBUG_PrintAllRanges(){
  Serial.println();
  Serial.println(F("------------------------------"));
  Serial.print(F("US: "));
  Serial.print(collisionObj.getUSRangeMM());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,L: "));
  Serial.print(collisionObj.getLSRRangeL());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,R: "));
  Serial.print(collisionObj.getLSRRangeR());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,A: "));
  Serial.print(collisionObj.getLSRRangeA());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,U: "));
  Serial.print(collisionObj.getLSRRangeU());
  Serial.println(F(" mm"));

  Serial.print(F("LSR,D: "));
  Serial.print(collisionObj.getLSRRangeD());
  Serial.println(F(" mm"));

  Serial.println(F("------------------------------"));
  Serial.println();
}

void DEBUG_PrintLightSens(){
  Serial.print("Light Sens L-");
  Serial.print("Lux: "); Serial.print(taskFindLightObj.getLuxLeft());
  Serial.print(", R-");
  Serial.print("Lux: "); Serial.println(taskFindLightObj.getLuxRight());
}

*/
