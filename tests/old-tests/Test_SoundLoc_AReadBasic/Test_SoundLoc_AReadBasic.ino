#define EARS_ENVSAMP 0
#define EARS_SNDLOC 1

#define EAR_L_AIN A8
#define EAR_R_AIN A9

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
uint16_t numEnvSamples = 200;
uint32_t LEnvSum = 0, REnvSum = 0;
int16_t envSampInd = 0;
// 12bit = 1845 average with DFRobot Fermion Mic
// 10bit = 460 average with DFRobot Fermion Mic
int16_t LEnvStd = 1845;
int16_t LEnvAvg = LEnvStd, REnvAvg = LEnvStd;
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
int16_t timeDiffMinMUS = 75, timeDiffMaxMUS = 350;

uint32_t tStart = 0, tEnd = 0;
uint32_t tElapsed = 0;
uint16_t numSamples = 1000;
int16_t sampVal = 0;

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  delay(2000);
}

void loop()
{

  if(modeFlag == EARS_SNDLOC){
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

void sampleEnv(){
  if((millis()-sampLastTime)>sampInterval){
    sampLastTime = millis();
    int16_t LSamp = analogRead(EAR_L_AIN);
    int16_t RSamp = analogRead(EAR_R_AIN);

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
    modeFlag = EARS_SNDLOC;

    LEnvAvg = int(float(LEnvSum)/float(numEnvSamples));
    REnvAvg = int(float(REnvSum)/float(numEnvSamples));
    LEnvSD = int(float(LEnvDiffSum)/float(numEnvSamples));
    REnvSD = int(float(REnvDiffSum)/float(numEnvSamples));
    
    soundLUpper = LEnvAvg + soundLOffset;
    soundRUpper = REnvAvg + soundROffset;
    soundLLower = LEnvAvg - soundLOffset;
    soundRLower = REnvAvg - soundROffset;

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
      // if greater than max time then this is a relfection
      earState = 1;
    }
    else if(soundLRTimeDiff >= timeDiffMinMUS){
      // if positive the right first
      earState = 3;
    }
    else if(soundLRTimeDiff <= -timeDiffMinMUS){
      // if negative, then left first
      earState = 2;
    }
    else{
      // if in the uncertainty range then go forward
      earState = 1;
    }
  }
  else if(foundSoundL || foundSoundR){
    if(foundSoundL){
      earState = 2;
    }
    else if(foundSoundR){
      earState = 3;
    }
    else{
      // If only one ear is tripped then go forward
      earState = 1;
    }
  }
  else{
    earState = 0;
  }

  if((millis()-earPrevResetTime) > earResetTime){
    earPrevResetTime = millis();

    Serial.print("STATE: ");
    Serial.print(earState);
    Serial.print(", FL: ");
    Serial.print(foundSoundL);
    Serial.print(", FR: ");
    Serial.print(foundSoundR);
    Serial.print(", TDiff: ");
    Serial.print(soundLRTimeDiff);
    Serial.print(", M: ");
    if(earState==1){Serial.print("F");}
    else if(earState==2){Serial.print("L");}
    else if(earState==3){Serial.print("R");}
    else{Serial.print("N");}
    Serial.println();

    // Reset Vars
    foundSoundBoth = false;
    foundSoundL = false;
    foundSoundR = false;
    soundLTimeStamp = 0;
    soundRTimeStamp = 0;
    soundLRTimeDiff = 0;
  }
}
