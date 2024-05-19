#define AINL A7
#define AINR A9

int16_t AL=0;
int16_t AR=0;

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop()
{  
  AL = analogRead(AINL);
  AR = analogRead(AINR);

  Serial.print(AL);
  Serial.print(",");
  Serial.print(AR);
  Serial.print(",");
  Serial.println();
}
