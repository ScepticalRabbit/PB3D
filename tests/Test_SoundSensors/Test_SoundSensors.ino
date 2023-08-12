#define AINL A1
#define AINR A3
#define DINL 0 
#define DINR 2

int16_t AL=0;
int16_t AR=0;
uint8_t DL=0;
uint8_t DR=0;

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(AINL, INPUT);
  pinMode(AINR, INPUT);
  pinMode(DINL, INPUT_PULLUP);
  pinMode(DINR, INPUT_PULLUP);
}

void loop()
{  
  AL = analogRead(AINL);
  DL = digitalRead(DINL);
  
  //AR = analogRead(AINR);
  //DR = digitalRead(DINR);
  
  Serial.print(DL*4000);
  Serial.print(",");
  Serial.println(AL);
  //Serial.print(",");
  //Serial.print(DR);
  //Serial.print(",");
  //Serial.println(AR);
  //Serial.println(",");
}
