# 1 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino"
//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

# 11 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 12 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2

// External Classes - Sensor and Module Libraries
# 15 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 16 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2

// Internal Classes - Basic Functions
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
# 32 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 33 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
// Internal Classes - Tasks
# 35 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 36 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 37 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 38 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 39 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 40 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 41 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 42 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 43 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2
# 44 "/home/lloydf/Arduino/PB3D/PB3D-core/PB3D-core.ino" 2

//-----------------------------------------------------------------------------
// VARIABLES
//-----------------------------------------------------------------------------

// Debugging - Codes
bool _debug_collisionOff = false;
bool _debug_forceMood = true;
int8_t _debug_moodCode = MOOD_NEUTRAL;
bool _debug_forceTask = true;
int8_t _debug_taskCode = 0;
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
  4, 6, ((1 << 6) | (1 << 4) | (0 << 2) | (2)) /*|< Transmit as G,R,B*/ + 0x0000 /*|< 800 KHz data transmission*/);

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
    moveObj.update_move(_debug_moveType);
  }
  taskObj.assignProb(moodObj.get_mood());

  tailObj.setState(0);
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
    taskObj.assignProb(moodObj.get_mood());
    moveObj.set_power_by_diff(moodObj.getPowerDiff());
    moveObj.set_speed_by_mood_fact(moodObj.getSpeedFact());
  }
  if(_debug_forceMood){moodObj.setMood(_debug_moodCode);}

  //----------------------------------------------------------------------------
  // GENERATE TASK - Based on internal timer, see Task class
  taskObj.update();

  // If task has changed update mood LEDs
  if(taskObj.getNewTaskFlag()){moodObj.reset_mood();}

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
  encoderL.update_speed();
  encoderR.update_speed();
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
  uint8_t inCodes[] = {0,0,0,0};
  speakerObj.setSoundCodes(inCodes,4);
  // Reset the tail to the central position - allow tasks and mood control
  tailObj.setState(0);

  //----------------------------------------------------------------------------
  // MAIN DECISION TREE
  //
  // NOTE: Rest and Interact are placed before collision avoidance to disable
  // handling of collisions while these modes are active - should be able to
  // make this smarter so certain things can re-enable collision avoidance
  if(taskObj.getTask() == -7){
    //DEBUG_SpeedTest(_test_value,MOVE_B_FORWARD);
  }
  else if(taskObj.getTask() == 1){
    taskRestObj.rest();
  }
  else if(taskObj.getTask() == -4 /* Only called by other tasks*/){
    taskPauseObj.pause();
  }
  else if(taskObj.getTask() == -3 /* Only called by other tasks*/){
    taskPickedUpObj.pickedUp();
  }
  else if(taskObj.getTask() == -2 /* Only called by other tasks*/){
    taskInteractObj.interact();
  }
  else if(collisionObj.get_escape_flag() && !_debug_collisionOff){
    //Serial.println("======================ESCAPE======================");
    escapeCollision(); // SEE FUNCTION DEF BELOW MAIN
  }
  else if(collisionObj.get_detected() && !_debug_collisionOff){
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
      moveObj.update_move(_debug_moveType);
    }
    else{
      moveObj.update_move();
    }
    // MOVEMENT: Call current movement function
    moveObj.go();
  }

  //-------------------------------------------------------------------------
  // POST DECISION TREE OVERRIDES
  // Wag tail if happy - regardless of task use of tail
  if(moodObj.get_mood() == MOOD_HAPPY){
    tailObj.setState(3);
    // wagmovetime,wagoffset,_test_pauseTime,wagcount
    tailObj.setWagParams(200,30,4000,6);
  }

  // If sleeping don't wag tail
  if(taskObj.getTask() == 1){
    tailObj.setState(0);
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
  encoderL.update_not_equal();
}
void updateEncLB(){
  encoderL.update_equal();
}
void updateEncRA(){
  encoderR.update_equal();
}
void updateEncRB(){
  encoderR.update_not_equal();
}

//------------------------------------------------------------------------------
// COLLISION HANDLING TASK TREE FUNCTIONS
void escapeCollision(){
  taskObj.taskLEDCollision();
  collisionObj.reset_flags();

  if(collisionObj.get_beepbeep_flag()){
    uint8_t inCodes[] = {2,2,0,0};
    speakerObj.setSoundCodes(inCodes,4);
  }

  collisionObj.escape();
}

void detectedCollision(){
  // Turn on collision LEDs and set escape flags
  taskObj.taskLEDCollision();
  collisionObj.set_escape_start();

  // If we are moving in a circle or a spiral then switch direction
  moveObj.change_circ_dir();

  // Reset the PIDs and stop the motors
  moveObj.stop();

  // If the bumper flag was tripped we need to go beep,beep!
  collisionObj.set_beepbeep_flag(false);
  if(collisionObj.get_bumper_flag()){
    collisionObj.set_beepbeep_flag(true);

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
    if(collisionObj.get_escape_turn() == MOVE_B_LEFT){
      taskPounceObj.setRealignCent(-1*angCent);
    }
    else{
      taskPounceObj.setRealignCent(1*angCent);
    }
  }

  // Reset the collision flags
  collisionObj.reset_flags();
  collisionObj.inc_count();
  if(collisionObj.get_count() >= taskTantrumObj.getThreshold()){
    collisionObj.reset_Count();
    taskTantrumObj.setStartTantrumFlag();
    taskObj.setTask(-1 /* Only called by other tasks*/);
  }
}

//------------------------------------------------------------------------------
// DEBUG FUNCTIONS
/*

void DEBUG_SpeedTest(uint8_t inPower, uint8_t moveCode){

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

        moveObj.back_power(inPower);

      }

      else if(moveCode == MOVE_B_LEFT){

        moveObj.left_power(inPower);

      }

      else if(moveCode == MOVE_B_RIGHT){

        moveObj.right_power(inPower);

      }

      else{

        moveObj.forward_power(inPower);

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

  Serial.print(encoderL.get_smooth_speed_mmps());

  Serial.print(F(" mm/s, R = "));

  Serial.print(encoderR.get_smooth_speed_mmps());

  Serial.print(F(" mm/s"));

  Serial.println();

}



void DEBUG_PrintSpeedCPS(){

  Serial.println();

  Serial.print(F("ENCODER SPEED, L="));

  Serial.print(encoderL.get_smooth_speed_mmps());

  Serial.print(F(" mm/s, R = "));

  Serial.print(encoderR.get_smooth_speed_mmps());

  Serial.print(F(" mm/s"));

  Serial.println();

}



void DEBUG_PlotSpeedBoth(){

  Serial.print(encoderL.get_smooth_speed_cps());

  Serial.print(",");

  Serial.print(encoderL.get_smooth_speed_mmps());

  Serial.print(",");

  Serial.print(encoderR.get_smooth_speed_cps());

  Serial.print(",");

  Serial.print(encoderR.get_smooth_speed_mmps());

  Serial.print(",");

  Serial.println();

}



void DEBUG_PlotSpeedPID_L(){

  Serial.print(moveObj.get_speed_PID_set_point_left());

  Serial.print(",");

  Serial.print(moveObj.get_speed_PID_output_left());

  Serial.print(",");

  Serial.print(moveObj.get_speed_PID_Pterm_left());

  Serial.print(",");

  Serial.print(moveObj.get_speed_PID_Iterm_left());

  Serial.print(",");

  Serial.print(moveObj.get_speed_PID_Dterm_left());

  Serial.print(",");

  Serial.print(encoderL.get_smooth_speed_mmps());

  Serial.print(",");

  Serial.println();

}



void DEBUG_PlotSpeedPID_R(){

  Serial.print(moveObj.get_speed_PID_set_point_right());

  Serial.print(",");

  Serial.print(moveObj.get_speed_PID_output_right());

  Serial.print(",");

  Serial.print(moveObj.get_speed_PID_Pterm_right());

  Serial.print(",");

  Serial.print(moveObj.get_speed_PID_Iterm_right());

  Serial.print(",");

  Serial.print(moveObj.get_speed_PID_Dterm_right());

  Serial.print(",");

  Serial.print(encoderR.get_smooth_speed_mmps());

  Serial.print(",");

  Serial.println();

}



void DEBUG_PlotSpeedMMPS(){

  uint32_t thisTime = millis();

  Serial.print(thisTime-_test_timeStamp);

  _test_timeStamp = thisTime;

  Serial.print(",");

  Serial.print(encoderL.get_smooth_speed_mmps());

  Serial.print(",");

  Serial.print(encoderR.get_smooth_speed_mmps());

  Serial.print(",");

  Serial.println();

}



void DEBUG_PrintColCheck(){

  Serial.println();

  Serial.println(F("------------------------------"));

  Serial.println("CheckVec=[BL,BR,US,LL,LR,LU,LD,]");

  Serial.print("CheckVec=[");

  for(uint8_t ii=0;ii<7;ii++){

      Serial.print(" ");Serial.print(collisionObj.get_col_check(ii));Serial.print(",");

  }

  Serial.println("]");



  Serial.print(F("US: "));

  //Serial.print(collisionObj.getColUSFlag());

  Serial.print(F(" , "));

  Serial.print(collisionObj.get_ultrasonic_range_mm());

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

  Serial.print(collisionObj.get_ultrasonic_range_mm());

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