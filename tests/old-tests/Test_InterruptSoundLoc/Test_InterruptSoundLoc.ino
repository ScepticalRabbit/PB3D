#define EAR_DINL 9
#define EAR_DINR 8

int32_t timeDiff = 0;
uint32_t timeStampL = 0;
uint32_t timeStampR = 0;
int16_t maxTime = 350; 
int32_t timeOut = 500000;

uint32_t reportTime = 1000;
uint32_t prevReportTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(EAR_DINL, INPUT_PULLUP);
  pinMode(EAR_DINR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EAR_DINL),earLeft,CHANGE);
  attachInterrupt(digitalPinToInterrupt(EAR_DINR),earRight,CHANGE);
}

void loop() {
  timeDiff = timeStampL - timeStampR;
  if(abs(timeDiff) > maxTime){
    timeDiff = 0;
  }
  
  if((millis()-prevReportTime) > reportTime){
    prevReportTime = millis();
    if(timeDiff < 0){
      Serial.print("LEFT, ");  
    }
    else if(timeDiff > 0){
      Serial.print("RIGHT, ");  
    }
    Serial.print("Time Diff: ");
    Serial.println(timeDiff);
  }
}

void earLeft(){
  if((micros() - timeStampL) > timeOut){
    timeStampL = micros();
  }
}

void earRight(){
  if((micros() - timeStampR) > timeOut){
    timeStampR = micros();
  }  
}
