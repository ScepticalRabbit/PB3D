# 1 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
//-----------------------------------------------------------------------------
// PB3D CORE PROGRAM
// Author: Lloyd Fletcher
// Version: v1.0
//-----------------------------------------------------------------------------

// Arduino Libraries
# 9 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 10 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2

// External Classes - Sensor and Module Libraries
# 13 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 14 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2

// Internal Classes - Basic Functions
# 17 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 18 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 19 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 20 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 21 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 22 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 23 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 24 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 25 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 26 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 27 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 28 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 29 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 30 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 31 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
// Internal Classes - Tasks
# 33 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 34 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 35 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 36 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 37 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 38 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 39 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 40 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 41 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 42 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2

//-----------------------------------------------------------------------------
// VARIABLES
//-----------------------------------------------------------------------------

// Debugging - Codes
bool _debug_collisionOff = false;
bool _debug_forceMood = true;
int8_t _debug_moodCode = 0;
bool _debug_forceTask = true;
int8_t _debug_taskCode = 0;
bool _debug_forceMove = false;
int8_t _debug_moveType = 2;

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
// EXTERNAL CLASSES - Adafruit and Grove

// MOOD LEDs
Adafruit_NeoPixel_ZeroDMA leds = Adafruit_NeoPixel_ZeroDMA(
  4, 6, ((1 << 6) | (1 << 4) | (0 << 2) | (2)) /*|< Transmit as G,R,B*/ + 0x0000 /*|< 800 KHz data transmission*/);

// MOTORS - Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//-----------------------------------------------------------------------------
// INTERNAL CLASSES

// SENSORS AND CONTROLLERS
// Digital pins for left and right encoders
static int encPinAL = 2, encPinBL = 3;
static int encPinAR = 4, encPinBR = 5;
// Declare encoder objects in main, pass pointers to move object
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
//-----------------------------------------------------------------------------
void setup() {
  // Start the serial
  Serial.begin(115200);
  // Only use below to stop start up until USB cable connected
  // while(!Serial){}

  // Initialize I2C communications for sensors and sub boards
  Wire.begin(); // Join I2C bus as leader
  delay(2000); // Needed to ensure sensors work, delay allows sub-processors to start up

  // SERIAL: POST SETUP
  Serial.println();
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(("PB3D: SETUP"))));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(("------------------------------"))));

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
  taskFindSoundObj.setSendByte(62);
  taskFindSoundObj.begin();

  // Pass durations to the master task object
  taskObj.setDanceDuration(taskDanceObj.getDuration());
  taskObj.setTantrumDuration(taskTantrumObj.getDuration());

  // Start timers in the main program
  _test_timer.start(0);
  _test_reportTimer.start(0);
  _test_pauseTimer.start(_test_pauseTime);

  // Encoders - Attach Interrupt Pins - CHANGE,RISING,FALLING
  attachInterrupt(( encPinAL ),
                  updateEncLA,2);
  attachInterrupt(( encPinAR ),
                  updateEncRA,2);
  attachInterrupt(( encPinBL ),
                  updateEncLB,2);
  attachInterrupt(( encPinBR ),
                  updateEncRB,2);

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

  tailObj.setState(0);
  //tailObj.setWagParams(uint16_t inMoveTime, int16_t inOffset, uint16_t inPauseTime, uint8_t inCountLim)
  Serial.println();

  // Final setup - increase I2C clock speed
  Wire.setClock(400000); // 400KHz

  //while(1){}
}

//---------------------------------------------------------------------------
// MAIN LOOP
//---------------------------------------------------------------------------
void loop(){
  //if(!_test_firstLoop){while(true){};}
  //uint32_t startLoop = millis();
  uint32_t startLoop = micros();

  //-------------------------------------------------------------------------
  // UPDATE MOOD - Based on internal timer, see Mood class
  moodObj.update();

  // If mood has updated then modify the other classes
  if(moodObj.getNewMoodFlag()){
    moodObj.setNewMoodFlag(false); // Reset the flag
    // Update task probabilities based on the current mood
    taskObj.assignProb(moodObj.getMood());
    // Update the movement speed based on the current mood
    moveObj.setPWRByDiff(moodObj.getPowerDiff());
    moveObj.setSpeedByMoodFact(moodObj.getSpeedFact());
  }
  if(_debug_forceMood){moodObj.setMood(_debug_moodCode);}

  //-------------------------------------------------------------------------
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
  if(_debug_collisionOff){collisionObj.setEnabledFlag(false);}

  //-------------------------------------------------------------------------
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

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // TEST CODE - SENSOR REPORTS
  if(_test_reportTimer.finished()){
    _test_reportTimer.start(_test_reportTime);
    //DEBUG_PrintColCheck();
    //DEBUG_PrintAllRanges();
    //DEBUG_PrintLightSens();
  }
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //-------------------------------------------------------------------------
  // RESET FLAGS
  //-------------------------------------------------------------------------
  // Reset the sound code and let the decision tree update it for the next loop
  uint8_t inCodes[] = {0,0,0,0};
  speakerObj.setSoundCodes(inCodes,4);
  // Reset the tail to the central position - allow tasks and mood control
  tailObj.setState(0);

  //-------------------------------------------------------------------------
  // MAIN DECISION TREE
  //-------------------------------------------------------------------------
  // NOTE: Rest and Interact are placed before collision avoidance to disable
  // handling of collisions while these modes are active - should be able to
  // make this smarter so certain things can re-enable collision avoidance
  if(taskObj.getTask() == -7){
    DEBUG_SpeedTest(_test_value,0);
  }
  else if(taskObj.getTask() == 1){
    taskRestObj.rest();
  }
  else if(taskObj.getTask() == -4 /* Only called by other tasks */){
    taskPauseObj.pause();
  }
  else if(taskObj.getTask() == -3 /* Only called by other tasks */){
    taskPickedUpObj.pickedUp();
  }
  else if(taskObj.getTask() == -2 /* Only called by other tasks */){
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
  else if(taskObj.getTask() == 2){
    if(taskObj.getDanceUpdateFlag()){
      taskObj.setDanceUpdateFlag(false);
      taskDanceObj.setSpeakerFlag(false);
    }
    taskDanceObj.dance();
  }
  else if(taskObj.getTask() == 3){
    taskFindHumanObj.findHuman();
  }
  else if(taskObj.getTask() == 5){
    taskFindLightObj.findLight();
  }
  else if(taskObj.getTask() == 6){
    taskFindLightObj.findDark();
  }
  else if(taskObj.getTask() == 4){
    taskFindSoundObj.findSound();
  }
  else if(taskObj.getTask() == 7){
    taskPounceObj.seekAndPounce();
  }
  else if(taskObj.getTask() == -1 /* Only called by other tasks*/){
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
  //-------------------------------------------------------------------------
  // Wag tail if happy - regardless of task use of tail
  if(moodObj.getMood() == 1){
    tailObj.setState(3);
    // wagmovetime,wagoffset,_test_pauseTime,wagcount
    tailObj.setWagParams(200,30,4000,6);
  }

  // If sleeping don't wag tail
  if(taskObj.getTask() == 1){
    tailObj.setState(0);
  }

  //-------------------------------------------------------------------------
  //uint32_t endLoop = millis();
  uint32_t endLoop = micros();
  //Serial.print(F("MAIN LOOP TOOK: "));
  //Serial.print(endLoop-startLoop); Serial.print(",");
  //Serial.print(endUpdate-startUpdate); Serial.print(",");
  //Serial.println();
  //_test_firstLoop = false;
}

//---------------------------------------------------------------------------
// INTERRUPT FUNCTIONS
//---------------------------------------------------------------------------
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

//---------------------------------------------------------------------------
// COLLISION HANDLING TASK TREE FUNCTIONS
//---------------------------------------------------------------------------
void escapeCollision(){
  taskObj.taskLEDCollision();
  collisionObj.resetFlags();

  if(collisionObj.getBeepBeepFlag()){
    uint8_t inCodes[] = {2,2,0,0};
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
    uint8_t inCodes[] = {2,2,0,0};
    speakerObj.setSoundCodes(inCodes,4);
    uint16_t inFreqs[] = {392,370,392,370,0,0,0,0};
    speakerObj.setSoundFreqs(inFreqs,8);
    uint16_t inDurs[] = {200,100,200,100,0,0,0,0};
    speakerObj.setSoundDurs(inDurs,8);
  }

  // Call specific tasks that need to handle collision events
  if((taskObj.getTask() == 5)||(taskObj.getTask() == 6)){
    taskFindLightObj.resetGrad();
  }
  if((taskObj.getTask() == 7) && (taskPounceObj.getState() == 2)){
    taskPounceObj.collisionResetToRealign();

    int16_t angCent = taskPounceObj.getAngCentForCollision();
    if(collisionObj.getEscapeTurn() == 3){
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
    taskObj.setTask(-1 /* Only called by other tasks*/);
  }
}

//---------------------------------------------------------------------------
// DEBUG FUNCTIONS
//---------------------------------------------------------------------------
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
      if(moveCode == 1){
        moveObj.backPWR(inPWR);
      }
      else if(moveCode == 3){
        moveObj.leftPWR(inPWR);
      }
      else if(moveCode == 4){
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
      if(moveCode == 1){
        moveObj.back(inSpeed);
      }
      else if(moveCode == 3){
        moveObj.left(inSpeed);
      }
      else if(moveCode == 4){
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
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("ENCODER SPEED, L="))));
  Serial.print(encoderL.getSmoothSpeedMMPS());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" mm/s, R = "))));
  Serial.print(encoderR.getSmoothSpeedMMPS());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" mm/s"))));
  Serial.println();
}

void DEBUG_PrintSpeedCPS(){
  Serial.println();
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("ENCODER SPEED, L="))));
  Serial.print(encoderL.getSmoothSpeedMMPS());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" mm/s, R = "))));
  Serial.print(encoderR.getSmoothSpeedMMPS());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" mm/s"))));
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
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(("------------------------------"))));
  Serial.println("CheckVec=[BL,BR,US,LL,LR,LU,LD,]");
  Serial.print("CheckVec=[");
  for(uint8_t ii=0;ii<7;ii++){
      Serial.print(" ");Serial.print(collisionObj.getColCheck(ii));Serial.print(",");
  }
  Serial.println("]");

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("US: "))));
  //Serial.print(collisionObj.getColUSFlag());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" , "))));
  Serial.print(collisionObj.getUSRangeMM());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,L: "))));
  //Serial.print(collisionObj.getColLSRFlagL());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" , "))));
  Serial.print(collisionObj.getLSRRangeL());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,R: "))));
  //Serial.print(collisionObj.getColLSRFlagR());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" , "))));
  Serial.print(collisionObj.getLSRRangeR());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,A: "))));
  //Serial.print(collisionObj.getColLSRFlagB());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" , "))));
  Serial.print(collisionObj.getLSRRangeA());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,U: "))));
  //Serial.print(collisionObj.getColLSRFlagU());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" , "))));
  Serial.print(collisionObj.getLSRRangeU());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,D: "))));
  //Serial.print(collisionObj.getColLSRFlagD());
  Serial.print((reinterpret_cast<const __FlashStringHelper *>((" , "))));
  Serial.print(collisionObj.getLSRRangeD());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.println((reinterpret_cast<const __FlashStringHelper *>(("------------------------------"))));
  Serial.println();
}

void DEBUG_PrintAllRanges(){
  Serial.println();
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(("------------------------------"))));
  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("US: "))));
  Serial.print(collisionObj.getUSRangeMM());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,L: "))));
  Serial.print(collisionObj.getLSRRangeL());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,R: "))));
  Serial.print(collisionObj.getLSRRangeR());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,A: "))));
  Serial.print(collisionObj.getLSRRangeA());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,U: "))));
  Serial.print(collisionObj.getLSRRangeU());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.print((reinterpret_cast<const __FlashStringHelper *>(("LSR,D: "))));
  Serial.print(collisionObj.getLSRRangeD());
  Serial.println((reinterpret_cast<const __FlashStringHelper *>((" mm"))));

  Serial.println((reinterpret_cast<const __FlashStringHelper *>(("------------------------------"))));
  Serial.println();
}

void DEBUG_PrintLightSens(){
  Serial.print("Light Sens L-");
  Serial.print("Lux: "); Serial.print(taskFindLightObj.getLuxLeft());
  Serial.print(", R-");
  Serial.print("Lux: "); Serial.println(taskFindLightObj.getLuxRight());
}
