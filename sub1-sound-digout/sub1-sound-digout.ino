//-----------------------------------------------------------------------------
// PB3D - Follower Board - Digital Out and Sound Loc
// Author: Lloyd Fletcher
// Version: v0.3a
// Date Created: 23rd May 2021
// Date Edited: 10th Dec 2022
//-----------------------------------------------------------------------------
/* 
TODO
*/ 
//-----------------------------------------------------------------------------

// Include Arduino Wire library for I2C
#include <Wire.h>

//---------------------------------------------------------------------------
// SENSOR SELECT
//#define EAR_GROVE
#define EAR_DFROB

#ifdef EAR_GROVE
  // 12bit = 1585 average with Grove mems mic
  int16_t LEnvAvgStd = 1585;
  // Pins for 'ears'
  #define EAR_L_AIN A7
  #define EAR_R_AIN A9
#endif

#ifdef EAR_DFROB
  // 12bit = 1845 average with DFRobot Fermion Mic
  // 10bit = 460 average with DFRobot Fermion Mic
  int16_t LEnvAvgStd = 1845;
  // Pins for 'ears'
  #define EAR_L_AIN A8
  #define EAR_R_AIN A9
#endif

//---------------------------------------------------------------------------
// I2C VARS
// Define follower I2C Address
//#define ADDR_MAINBOARD 0x12
#define ADDR_FOLLBOARD 0x11

// Digital pins for output
#define DOUT_PURR  0
#define DOUT_LSR_L 1
#define DOUT_LSR_R 2
#define DOUT_LSR_B 3
#define DOUT_LSR_U 6
#define DOUT_LSR_D 7

// Byte to receive from main board 
byte recByte = B00000000;

//---------------------------------------------------------------------------
// SOUND LOC/EAR VARS
// Codes for ear state
#define EAR_MODE_ENVSAMP 0
#define EAR_MODE_SNDLOC 1

#define EAR_COM_NOSOUND 0
#define EAR_COM_FORWARD 1
#define EAR_COM_LEFT 2
#define EAR_COM_RIGHT 3
#define EAR_COM_SENV 4

// Mode 0 = sample environment
// Mode 1 = locate sound
uint8_t modeFlag = 0;

// Variables for sound location code
// 0: no sound
// 1: forward -> sound but uncertain direction
// 2: left
// 3: right
byte earState = 0; // byte to send 

uint16_t sampInterval = 10;
uint32_t sampLastTime = 0;
uint16_t numEnvSamples = 100;
uint32_t LEnvSum = 0, REnvSum = 0;
int16_t envSampInd = 0;
int16_t LEnvAvg = LEnvAvgStd, REnvAvg = LEnvAvgStd;
int16_t LEnvDiffSum = 0, REnvDiffSum = 0; 
int16_t LEnvSD = 0, REnvSD = 0; 

bool foundSoundBoth = false;
int32_t soundLRTimeDiff = 0;

bool foundSoundL = false, foundSoundR = false;
uint32_t soundLTimeStamp = 0, soundRTimeStamp = 0;
int16_t soundL = 0, soundR = 0;
int16_t soundLOffset = 500, soundROffset = 500;

uint32_t earResetTime = 500;
uint32_t earPrevResetTime = 0;

int16_t soundLUpper = LEnvAvg + soundLOffset;
int16_t soundRUpper = REnvAvg + soundROffset;
int16_t soundLLower = LEnvAvg - soundLOffset;
int16_t soundRLower = REnvAvg - soundROffset;
int16_t timeDiffMinMUS = 75, timeDiffMaxMUS = 400;

uint32_t tStart = 0, tEnd = 0;
uint32_t tElapsed = 0;
uint16_t numSamples = 1000;
int16_t sampVal = 0;

byte prevByte = B00000000;

//---------------------------------------------------------------------------
// SETUP
void setup(){
  // Initialize I2C communications
  Wire.begin(ADDR_FOLLBOARD);
  // Function to run when data received from leader
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  // Setup Serial Monitor 
  Serial.begin(115200);

  // Setup Digital Output Pins
  pinMode(DOUT_PURR,OUTPUT);
  pinMode(DOUT_LSR_L,OUTPUT);
  pinMode(DOUT_LSR_R,OUTPUT);
  pinMode(DOUT_LSR_B,OUTPUT);
  pinMode(DOUT_LSR_U,OUTPUT);
  pinMode(DOUT_LSR_D,OUTPUT);
  // Set all pins low
  digitalWrite(DOUT_PURR,LOW);
  digitalWrite(DOUT_LSR_L,LOW);
  digitalWrite(DOUT_LSR_R,LOW);
  digitalWrite(DOUT_LSR_B,LOW);
  digitalWrite(DOUT_LSR_U,LOW);
  digitalWrite(DOUT_LSR_D,LOW);

  // Set the analog read resolution for SAMD51
  analogReadResolution(12);

  /*
  delay(2000);
  Serial.println();
  Serial.println("----------------------------------------");
  Serial.println("SETUP DEBUG");
  */
}

//---------------------------------------------------------------------------
// MAIN LOOP
void loop(){
  //---------------------------------------------------------------------------
  // DIGITAL OUTPUTS
  //---------------------------------------------------------------------------
  // Vibration Motor - Purring
  if((recByte & B00000001) == B00000001){
    digitalWrite(DOUT_PURR,HIGH);
  }
  else{
    digitalWrite(DOUT_PURR,LOW);
  }
  // Left Laser Sensor
  if((recByte & B00000010) == B00000010){
    digitalWrite(DOUT_LSR_L,HIGH);
  }
  else{
    digitalWrite(DOUT_LSR_L,LOW);
  }
  // Right Laser Sensor
  if((recByte & B00000100) == B00000100){
    digitalWrite(DOUT_LSR_R,HIGH);
  }
  else{
    digitalWrite(DOUT_LSR_R,LOW);
  }
  // Bottom Laser Sensor
  if((recByte & B00001000) == B00001000){
    digitalWrite(DOUT_LSR_B,HIGH);
  }
  else{
    digitalWrite(DOUT_LSR_B,LOW);
  }
  // Up Laser Sensor
  if((recByte & B00010000) == B00010000){
    digitalWrite(DOUT_LSR_U,HIGH);
  }
  else{
    digitalWrite(DOUT_LSR_U,LOW);
  }
  // Down Laser Sensor
  if((recByte & B00100000) == B00100000){
    digitalWrite(DOUT_LSR_D,HIGH);
  }
  else{
    digitalWrite(DOUT_LSR_D,LOW);
  }

  //---------------------------------------------------------------------------
  // SOUND LOCATION
  //---------------------------------------------------------------------------
  // HACK: for some reason the recByte keeps being set to 255 (B11111111)
  // So we ignore this case here to prevent errors
  if((recByte != B11111111) && ((recByte & B01000000) == B01000000)){
    modeFlag = EAR_MODE_ENVSAMP;  
  }
  
  if(modeFlag == EAR_MODE_SNDLOC){
    locSound();
  }
  else{
    sampleEnv();
  }

  //tttttttttttttttttttttttttttttttttttttttttttttttt
  // TIMING CODE
  /*
  tStart = micros();
  for(int i=0; i<numSamples; i++) {
    foundSoundL = true;
    locSound();
  }
  tEnd = micros();
  tElapsed = tEnd - tStart;
  float tAvgMicroS = float(tElapsed)/float(numSamples);
  float tAvgS = tAvgMicroS/1000000.0;
  float freqAvg = 1.0/tAvgS;   

  Serial.print("Avg. Time per Sample: ");
  Serial.print(tAvgMicroS);
  Serial.println(" us");
  Serial.print("Avg. Frequency: ");
  Serial.print(freqAvg);
  Serial.println(" Hz");
  Serial.println();
  delay(5000);
  */
  //tttttttttttttttttttttttttttttttttttttttttttttttt
  
}

//---------------------------------------------------------------------------
// FUNCTIONS - I2C COMMS
void receiveEvent(int numBytes) {
  recByte = Wire.read();
}

void requestEvent() {
  Wire.write(earState);
}

//---------------------------------------------------------------------------
// FUNCTIONS - SOUND LOCATION
void sampleEnv(){
  earState = EAR_COM_SENV;
  
  if((millis()-sampLastTime)>sampInterval){
    sampLastTime = millis();
    int16_t LSamp = analogRead(EAR_L_AIN);
    int16_t RSamp = analogRead(EAR_R_AIN);
    //int16_t RSamp = 0;
    
    // Sum of values to calculate average
    LEnvSum = LEnvSum + LSamp;
    REnvSum = REnvSum + RSamp;

    // Difference to prelim average for SD calc
    LEnvDiffSum = LEnvDiffSum + abs(LEnvAvg - LSamp);
    REnvDiffSum = REnvDiffSum + abs(REnvAvg - RSamp);
    
    envSampInd++;
  }

  if(envSampInd >= numEnvSamples){
    envSampInd = 0;
    modeFlag = EAR_MODE_SNDLOC;
    earState = EAR_COM_NOSOUND;

    LEnvAvg = int(float(LEnvSum)/float(numEnvSamples));
    REnvAvg = int(float(REnvSum)/float(numEnvSamples));
    LEnvSD = int(float(LEnvDiffSum)/float(numEnvSamples));
    REnvSD = int(float(REnvDiffSum)/float(numEnvSamples));

    if(6*LEnvSD >= soundLOffset){soundLOffset = 6*LEnvSD;}
    if(6*REnvSD >= soundROffset){soundROffset = 6*REnvSD;}
    
    soundLUpper = LEnvAvg + soundLOffset;
    soundRUpper = REnvAvg + soundROffset;
    soundLLower = LEnvAvg - soundLOffset;
    soundRLower = REnvAvg - soundROffset;
    /*
    Serial.println();
    Serial.println("----------------------------------------");
    Serial.println("EAR-ENVSAMP");
    Serial.print("L Sum: ");
    Serial.print(LEnvSum);
    Serial.print(" , ");
    Serial.print("R Sum: ");
    Serial.println(REnvSum);
    Serial.print("L Avg: ");
    Serial.print(LEnvAvg);
    Serial.print(" , ");
    Serial.print("R Avg: ");
    Serial.println(REnvAvg);
    Serial.print("L SD: ");
    Serial.print(LEnvSD);
    Serial.print(" , ");
    Serial.print("R SD: ");
    Serial.println(REnvSD);
    Serial.print("L 6xSD: ");
    Serial.print(6*LEnvSD);
    Serial.print(" , ");
    Serial.print("R 6xSD: ");
    Serial.println(6*REnvSD);
    */
    // Reset environment calc sums
    LEnvSum = 0, REnvSum = 0;
    LEnvDiffSum = 0, REnvDiffSum = 0;
  }
}

void locSound(){
  if(!foundSoundL){
    soundL = analogRead(EAR_L_AIN);
    if(((soundL > soundLUpper) || (soundL < soundLLower))){
      soundLTimeStamp = micros();
      foundSoundL = true;
    }
  }

  if(!foundSoundR){
    soundR = analogRead(EAR_R_AIN);
    if(!foundSoundR && ((soundR > soundRUpper) || (soundR < soundRLower))){
      soundRTimeStamp = micros();
      foundSoundR = true;
    }
  }

  if(foundSoundL && foundSoundR){
    foundSoundBoth = true;
    soundLRTimeDiff = soundLTimeStamp - soundRTimeStamp;

    if(abs(soundLRTimeDiff) > timeDiffMaxMUS){
      // if greater than max time then this is a reflection
      earState = EAR_COM_FORWARD;
    }
    else if(soundLRTimeDiff >= timeDiffMinMUS){
      // if positive the right first
      earState = EAR_COM_RIGHT;
    }
    else if(soundLRTimeDiff <= -timeDiffMinMUS){
      // if negative, then left first
      earState = EAR_COM_LEFT;
    }
    else{
      // if in the uncertainty range then go forward
      earState = EAR_COM_FORWARD;
    }
  }
  else if(foundSoundL || foundSoundR){
    if(foundSoundL){
      earState = EAR_COM_LEFT;
    }
    else if(foundSoundR){
      earState = EAR_COM_RIGHT;
    }
    else{
      // If only one ear is tripped then go forward
      earState = EAR_COM_FORWARD;
    }
  }
  else{
    earState = EAR_COM_NOSOUND;
  }

  if((millis()-earPrevResetTime) > earResetTime){
    earPrevResetTime = millis();

    /*
    Serial.print("STATE: ");
    Serial.print(earState);
    Serial.print(", FL: ");
    Serial.print(foundSoundL);
    Serial.print(", FR: ");
    Serial.print(foundSoundR);
    Serial.print(", TDiff: ");
    Serial.print(soundLRTimeDiff);
    Serial.print(", M: ");
    if(_earState==EAR_COM_FORWARD){Serial.print("F");}
    else if(_earState==EAR_COM_LEFT){Serial.print("L");}
    else if(_earState==EAR_COM_RIGHT){Serial.print("R");}
    else if(_earState==EAR_COM_SENV){Serial.print("E");}
    else{Serial.print("N");}
    Serial.println();
    */
    // Reset Vars
    foundSoundBoth = false;
    foundSoundL = false;
    foundSoundR = false;
    soundLTimeStamp = 0;
    soundRTimeStamp = 0;
    soundLRTimeDiff = 0;
  }
}
