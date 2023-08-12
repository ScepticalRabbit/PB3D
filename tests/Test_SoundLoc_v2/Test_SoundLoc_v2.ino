#define EARS_ENVSAMP 0
#define EARS_SNDLOC 1

#define EAR_L_AIN A0
#define EAR_R_AIN A2

uint8_t modeFlag = 0;

uint32_t sampleEnvTime = 120000;
uint16_t sampleInterval = 10;
uint16_t numEnvSamples = 100;
uint32_t LEnvSum = 0, REnvSum = 0;
int16_t LEnvAvg = 0, REnvAvg = 0;
int16_t envSampInd = 0;

bool foundSoundBoth = false;
int32_t soundLRTimeDiff = 0;

bool foundSoundL = false;
uint32_t soundLTimeStamp = 0;
int16_t soundL = 0;
int16_t soundLUpper = 0, soundLLower = 0;
int16_t soundLOffset = 500;

bool foundSoundR = false;
uint32_t soundRTimeStamp = 0;
int16_t soundR = 0;
int16_t soundRUpper = 0, soundRLower = 0;
int16_t soundROffset = 500;

uint32_t sampleTimeMUS = 300;
uint32_t endSampleMUS = 0;

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(EAR_L_AIN, INPUT);
  pinMode(EAR_R_AIN, INPUT);
  delay(1000);
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
  envSampInd++;

  if(envSampInd >= numEnvSamples){
    envSampInd = 0;
    modeFlag = EARS_SNDLOC;

    LEnvAvg = int(float(LEnvSum)/float(numEnvSamples));
    REnvAvg = int(float(REnvSum)/float(numEnvSamples));
    soundLUpper = LEnvAvg + soundLOffset;
    soundRUpper = REnvAvg + soundROffset;
    soundLLower = LEnvAvg - soundLOffset;
    soundRLower = REnvAvg - soundROffset;
    //soundLThres = soundLOffset;
    //soundRThres = soundROffset;

    Serial.print("L Avg: ");
    Serial.print(LEnvAvg);
    Serial.print(" , ");
    Serial.print("R Avg: ");
    Serial.println(REnvAvg);
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

  if(foundSoundL && foundSoundR){
    foundSoundBoth = true;
    soundLRTimeDiff = soundLTimeStamp - soundRTimeStamp;

    Serial.print("LR Diff: ");
    Serial.print(soundLRTimeDiff);
    Serial.print(", ");
    if(soundLRTimeDiff >= 0){
      Serial.println("RIGHT ear");
    }
    else if(soundLRTimeDiff <= 0){
      Serial.println("LEFT ear");
    }
    else{
      Serial.println("ERROR!");
    }
    Serial.println();

    delay(100);
  }
  
  if(micros() > endSampleMUS){
    endSampleMUS = micros()+sampleTimeMUS;
    foundSoundBoth = false;
    foundSoundL = false;
    foundSoundR = false;
    soundLTimeStamp = 0;
    soundRTimeStamp = 0;
  }
}
