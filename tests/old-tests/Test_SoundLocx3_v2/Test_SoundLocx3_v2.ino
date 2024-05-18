#define EARS_ENVSAMP 0
#define EARS_SNDLOC 1

#define EAR_L_AIN A0
#define EAR_R_AIN A1
#define EAR_C_AIN A2

uint8_t modeFlag = 0;

uint32_t sampleEnvTime = 120000;
uint16_t sampleInterval = 10;
uint16_t numEnvSamples = 100;
uint32_t LEnvSum = 0, REnvSum = 0, CEnvSum = 0;
int16_t LEnvAvg = 0, REnvAvg = 0, CEnvAvg = 0;
int16_t envSampInd = 0;

bool foundSoundBoth = false;
int32_t soundLRTimeDiff = 0;
int32_t soundCLTimeDiff = 0;
int32_t soundCRTimeDiff = 0;

int16_t soundMainOffset = 120;

bool foundSoundL = false;
uint32_t soundLTimeStamp = 0;
int16_t soundL = 0;
int16_t soundLUpper = 0, soundLLower = 0;
int16_t soundLOffset = soundMainOffset;

bool foundSoundR = false;
uint32_t soundRTimeStamp = 0;
int16_t soundR = 0;
int16_t soundRUpper = 0, soundRLower = 0;
int16_t soundROffset = soundMainOffset;

bool foundSoundC = false;
uint32_t soundCTimeStamp = 0;
int16_t soundC = 0;
int16_t soundCUpper = 0, soundCLower = 0;
int16_t soundCOffset = soundMainOffset;

uint32_t sampleTimeMUS = 300;
int16_t timeUncertMUS = 100;
uint32_t endSampleMUS = 0;

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(EAR_L_AIN, INPUT);
  pinMode(EAR_R_AIN, INPUT);
  pinMode(EAR_C_AIN, INPUT);
  delay(4000);
}

void loop()
{
  if(modeFlag == EARS_SNDLOC){
    locSound();
  }
  else{
    sampleEnv();
  }
}

void sampleEnv(){
  LEnvSum = LEnvSum + analogRead(EAR_L_AIN);
  REnvSum = REnvSum + analogRead(EAR_R_AIN);
  CEnvSum = CEnvSum + analogRead(EAR_C_AIN);
  envSampInd++;

  if(envSampInd >= numEnvSamples){
    envSampInd = 0;
    modeFlag = EARS_SNDLOC;

    LEnvAvg = int(float(LEnvSum)/float(numEnvSamples));
    soundLUpper = LEnvAvg + soundLOffset;
    soundLLower = LEnvAvg - soundLOffset;
    
    REnvAvg = int(float(REnvSum)/float(numEnvSamples));
    soundRUpper = REnvAvg + soundROffset;
    soundRLower = REnvAvg - soundROffset;

    CEnvAvg = int(float(CEnvSum)/float(numEnvSamples));
    soundCUpper = CEnvAvg + soundCOffset;
    soundCLower = CEnvAvg - soundCOffset;

    Serial.print("L Avg: ");
    Serial.print(LEnvAvg);
    Serial.print(" , ");
    Serial.print("R Avg: ");
    Serial.print(REnvAvg);
    Serial.print(" , ");
    Serial.print("C Avg: ");
    Serial.print(CEnvAvg);
    Serial.println();
  }
}

void locSound(){
  soundL = analogRead(EAR_L_AIN);
  if(!foundSoundL && ((soundL > soundLUpper) || (soundL < soundLLower))){
    soundLTimeStamp = micros();
    foundSoundL = true;
  }
  
  soundR = analogRead(EAR_R_AIN);
  if(!foundSoundR && ((soundR > soundRUpper) || (soundR < soundRLower))){
    soundRTimeStamp = micros();
    foundSoundR = true;
  }

 soundC = analogRead(EAR_C_AIN);
  if(!foundSoundC && ((soundC > soundCUpper) || (soundC < soundCLower))){
    soundCTimeStamp = micros();
    foundSoundC = true;
  }

  if(foundSoundL && foundSoundR && foundSoundC){
    foundSoundBoth = true;
    soundLRTimeDiff = soundLTimeStamp - soundRTimeStamp;
    soundCLTimeDiff = soundCTimeStamp - soundLTimeStamp;
    soundCRTimeDiff = soundCTimeStamp - soundRTimeStamp;

    Serial.print("LR Diff: ");
    Serial.print(soundLRTimeDiff);
    Serial.print(", ");
    if(soundLRTimeDiff >= timeUncertMUS){
      Serial.println("RIGHT ear");
    }
    else if(soundLRTimeDiff <= -timeUncertMUS){
      Serial.println("LEFT ear");
    }
    else{
      Serial.println("UNKNOWN");
    }

    Serial.print("CL Diff: ");
    Serial.print(soundCLTimeDiff);
    Serial.print(", ");
     
    if(soundCLTimeDiff >= timeUncertMUS){
      Serial.println("LEFT first");
    }  
    else if(soundCLTimeDiff <= -timeUncertMUS){
      Serial.println("CENT first");
    }
    else{
      Serial.println("UNKNOWN");
    }
    
    Serial.print("CR Diff: ");
    Serial.print(soundCRTimeDiff);
    Serial.print(", ");
    if(soundCRTimeDiff >= timeUncertMUS){
      Serial.println("RIGHT first");
    }
    else if(soundCRTimeDiff <= -timeUncertMUS){
      Serial.println("CENT first");
    }
    else{
      Serial.println("UNKNOWN");
    }
    Serial.println();

    delay(100);
  }
  
  if(micros() > endSampleMUS){
    endSampleMUS = micros()+sampleTimeMUS;
    foundSoundBoth = false;
    foundSoundL = false;
    foundSoundR = false;
    foundSoundC = false;
    soundLTimeStamp = 0;
    soundRTimeStamp = 0;
    soundCTimeStamp = 0;
  }
}
