// Digital pins for output
#define DOUT_PURR  7

bool purrSwitch = true;
uint16_t purrInt = 1000;
uint32_t lastTime = 0;

void setup() {
  // Setup Serial Monitor 
  Serial.begin(115200);
  Serial.println("TEST VIBE MOTOR");

  // Setup Digital Output Pins
  pinMode(DOUT_PURR,OUTPUT);
}

void loop() {
  if((millis()-lastTime) > purrInt){
    lastTime = millis();
    purrSwitch = !purrSwitch;
    
    if(purrSwitch){
      digitalWrite(DOUT_PURR,HIGH);
    }
    else{
      digitalWrite(DOUT_PURR,LOW);
    }  
  }
}
