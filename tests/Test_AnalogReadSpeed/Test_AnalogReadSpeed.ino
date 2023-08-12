#define READAIN1 8
#define READAIN2 9

uint32_t tStart = 0, tEnd = 0;
uint32_t tElapsed = 0;
uint16_t numSamples = 1000;
int16_t sampVal = 0;

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);  
}

void loop()
{
  tStart = micros();
  for(int i=0; i<numSamples; i++) {
    //sampVal = analogRead(READAIN1);
    //sampVal = analogRead(READAIN2);
    analogRead(READAIN1);
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
}
