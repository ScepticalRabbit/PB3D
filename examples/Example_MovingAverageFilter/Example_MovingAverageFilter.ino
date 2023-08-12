//Parameters
const int aisPin  = A0;
const int numReadings  = 10;
int readings [numReadings];
int readIndex  = 0;
long total  = 0;

//Variables
int aisVal  = 0;

void setup() {
  //Init Serial USB
  Serial.begin(9600);
  Serial.println(F("Initialize System"));
  //Init AnalogSmooth
  pinMode(aisPin, INPUT);
}

void loop() {
  readAnalogSmooth();
  Serial.print(F("ais avg : ")); Serial.println(smooth());
  delay(200);
}

void readAnalogSmooth( ) { /* function readAnalogSmooth */
  ////Test routine for AnalogSmooth
  aisVal = analogRead(aisPin);
  Serial.print(F("ais val ")); Serial.println(aisVal);
}

long smooth() { /* function smooth */
  ////Perform average on sensor readings
  long average;
  // subtract the last reading:
  total = total - readings[readIndex];
  // read the sensor:
  readings[readIndex] = analogRead(aisPin);
  // add value to total:
  total = total + readings[readIndex];
  // handle index
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;

  return average;
}
