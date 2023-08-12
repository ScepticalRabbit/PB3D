#include "Timer.h"

#define AINL A0
#define AINR A1


int16_t SL=0;
int16_t SR=0;
int16_t SLMax = 0;
int16_t SRMax = 0;

Timer reportTimer = Timer();
uint16_t reportTime = 4000;

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(AINL, INPUT);
  pinMode(AINL, INPUT);

  reportTimer.start(reportTime);
}

void loop()
{  
  SL = analogRead(AINL);
  SR = analogRead(AINL);

  if(SL > SLMax){
    SLMax = SL;
  }
  if(SR > SRMax){
    SRMax = SR;
  }
  
  if(reportTimer.finished()){
    reportTimer.start(reportTime);
    
    Serial.print("L Max ");
    Serial.print(SLMax);
    Serial.print(" , ");
    Serial.print("R Max ");
    Serial.println(SRMax);

    SLMax = 0;
    SRMax = 0;
  }
}
