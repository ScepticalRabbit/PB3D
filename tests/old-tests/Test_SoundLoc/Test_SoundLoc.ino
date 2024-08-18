#include "Timer.h"
#include "MicroTimer.h"

#define EARS_ENVSAMP 0
#define EARS_SNDLOC 1

#define EAR_N -1
#define EAR_L 0
#define EAR_R 1

#define EAR_L_AIN A1
#define EAR_R_AIN A2

uint8_t modeFlag = 0;

Timer envUpdateTimer = Timer();
Timer envSampleTimer = Timer();
bool envUpdateOn = false;
uint32_t sampleEnvTime = 10000;
uint16_t sampleInterval = 1;
const uint16_t numEnvSamples = 200;
uint32_t LEnvSum = 0, REnvSum = 0;
int16_t LEnvAvg = 0, REnvAvg = 0;
int16_t envSampInd = 0;

uint16_t locSoundInt = 2500;
Timer locSoundUpdateTimer = Timer();

int8_t firstEar = EAR_N;
bool soundFound = false;

bool locSoundL = false;
int16_t soundL = 0;
int16_t soundLThres = 0;
int16_t soundLOffset = 300;

bool locSoundR = false;
int16_t soundR = 0;
int16_t soundRThres = 0;
int16_t soundROffset = 300;

int16_t earSpacingMm = 100;
int16_t earTimeOutMUS = 300;

uint32_t soundDiffTimeMUS = 0;
MicroTimer locSoundMicroTimer = MicroTimer();

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(EAR_L_AIN, INPUT);
  pinMode(EAR_R_AIN, INPUT);

  envUpdateTimer.start(0);
  envSampleTimer.start(0);
  locSoundUpdateTimer.start(0);
  locSoundMicroTimer.start(0);
}

void loop()
{
  sampleEnv();
  locSound();
}

void sampleEnv(){
  if(!envUpdateOn && envUpdateTimer.finished()){
    envUpdateTimer.start(sampleEnvTime);

    envSampleTimer.start(0);
    envUpdateOn = true;
    modeFlag = EARS_ENVSAMP;

    LEnvSum = 0;
    REnvSum = 0;
  }

  if(envUpdateOn && envSampleTimer.finished()){
    envSampleTimer.start(sampleInterval);

    LEnvSum = LEnvSum + analogRead(EAR_L_AIN);
    REnvSum = REnvSum + analogRead(EAR_R_AIN);
    envSampInd++;

    if(envSampInd >= numEnvSamples){
      envSampInd = 0;
      envUpdateOn = false;
      modeFlag = EARS_SNDLOC;

      LEnvAvg = int(float(LEnvSum)/float(numEnvSamples));
      REnvAvg = int(float(REnvSum)/float(numEnvSamples));
      //soundLThres = LEnvAvg + soundLOffset;
      //soundRThres = REnvAvg + soundROffset;
      soundLThres = soundLOffset;
      soundRThres = soundROffset;

      Serial.print("L Avg: ");
      Serial.print(LEnvAvg);
      Serial.print(" , ");
      Serial.print("R Avg: ");
      Serial.println(REnvAvg);
      Serial.println();
    }
  }
}

void locSound(){
  if(modeFlag == EARS_SNDLOC){
    if(locSoundUpdateTimer.finished()){
      locSoundUpdateTimer.start(locSoundInt);

      Serial.print("F: ");
      Serial.print(soundFound);
      Serial.print(", FE: ");
      Serial.print(firstEar);
      Serial.print(", SDT: ");
      Serial.println(soundDiffTimeMUS);
      Serial.println();

      soundFound = false;
      locSoundL = false;
      locSoundR = false;
      soundDiffTimeMUS = 0;
      firstEar = EAR_N;
    }

    if(!locSoundL && !locSoundR){
      //soundL = analogRead(EAR_L_AIN);
      //soundR = analogRead(EAR_R_AIN);
      soundL = abs(analogRead(EAR_L_AIN)-LEnvAvg);
      soundR = abs(analogRead(EAR_R_AIN)-REnvAvg);

      if(soundL > soundLThres){
        locSoundMicroTimer.start(earTimeOutMUS);
        locSoundL = true;
        firstEar = EAR_L;

        locSoundMicroTimer.start(earTimeOutMUS);
        while(true){
          if(!locSoundMicroTimer.finished()){
            //soundR = analogRead(EAR_R_AIN);
            soundR = abs(analogRead(EAR_R_AIN)-REnvAvg);
            if(soundR > soundRThres){
              soundDiffTimeMUS = locSoundMicroTimer.get_time();
              locSoundR = true;
              soundFound = true;
            }
          }
          else{
            locSoundL = false;
            locSoundR = false;
            break;
          }
        }
      }
      else if(soundR > soundRThres){
        locSoundMicroTimer.start(earTimeOutMUS);
        locSoundR = true;
        firstEar = EAR_R;

        while(true){
          if(!locSoundMicroTimer.finished()){
            //soundL = analogRead(EAR_L_AIN);
            soundL = abs(analogRead(EAR_L_AIN)-LEnvAvg);
            if(soundL > soundLThres){
              soundDiffTimeMUS = locSoundMicroTimer.get_time();
              locSoundL = true;
              soundFound = true;
            }
          }
          else{
            locSoundL = false;
            locSoundR = false;
            break;
          }
        }
      }
    }
  }
}
