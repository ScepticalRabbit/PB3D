#include <Wire.h>

#define BUMPER_FR 0
#define BUMPER_FL 1
#define BUMPER_BR 8
#define BUMPER_BL 9
#define IR_R 2
#define IR_L 7


void setup() {
  Serial.begin(115200);
   
  pinMode(BUMPER_FR, INPUT_PULLUP);
  pinMode(BUMPER_FL, INPUT_PULLUP);  
  pinMode(BUMPER_BR, INPUT_PULLUP);
  pinMode(BUMPER_BL, INPUT_PULLUP);
  pinMode(IR_R, INPUT);
  pinMode(IR_L, INPUT);  
}

void loop() {
  int8_t switchFR = digitalRead(BUMPER_FR);
  int8_t switchFL = digitalRead(BUMPER_FL);
  int8_t switchBR = digitalRead(BUMPER_BR);
  int8_t switchBL = digitalRead(BUMPER_BL);
  int8_t IRL = digitalRead(IR_L);
  int8_t IRR = digitalRead(IR_R);

  Serial.print("FL: ");
  Serial.print(switchFL);
  Serial.print(", FR: ");
  Serial.print(switchFR);
  Serial.print(", BL: ");
  Serial.print(switchBL);
  Serial.print(", BR: ");
  Serial.print(switchBR);
  Serial.print(", IRL: ");
  Serial.print(IRL);
  Serial.print(", IRR: ");
  Serial.print(IRR);
  Serial.println("");
}
