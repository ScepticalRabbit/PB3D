#include <Wire.h>
#include <Adafruit_MotorShield.h>

// NOTES
/*
 * Pololu encode = 12 counts per revolution =>
 * Multiply gear ratio by 12 to get counts per wheel rev
 * Micrometal motors are 50:1
 * Total Counts Per Rev = 12 x 50 (gear ratio)
 * Total Counts Per Rev = 600
 *
 * Largest storable enoder count:
 * -2,147,483,648 to 2,147,483,647
 * With 600 counts per rev = 3579139 revs
 * Wheel diameter is 42mm = 42*pi = 131.9mm per rev
 * Total travel dist = 131.9mm per rev x 3579139 revs
 * Total travel dist = 472,088,434 mm = 472 km
 */

// Create the motor shield object with the default I2C address
Adafruit_MotorShield motor_shield = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield motor_shield = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotorR = motor_shield.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myMotorL = motor_shield.getMotor(2);

char commandDir = 'F';
// Gear ratio = 50, Encoder has 12 counts per rev
// Total of 600 counts per rev
static int pinAR = 4; // Our first hardware interrupt pin is digital pin 2
static int pinBR = 5; // Our second hardware interrupt pin is digital pin 3
volatile long encCounterR = 0;
volatile bool currStateAR = false;
volatile bool lastStateAR = false;
char encDirR = 'F';
long oldEncCounterR = 0;

static int pinAL = 2;
static int pinBL = 3;
volatile long encCounterL = 0;
volatile bool currStateAL = false;
volatile bool lastStateAL = false;
char encDirL = 'F';
long oldEncCounterL = 0;

uint8_t maxMotorSpeed = 200;
uint16_t speedUpdateTime = 20;
uint16_t pauseTime = 1000;

bool printLeft = true;
bool printRight = false;
bool runLeft = true;
bool runRight = false;

void updateEncoderR();
void updateEncoderL();

void setup() {
  pinMode(pinAR, INPUT); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinBR, INPUT); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(pinAR),updateEncoderR,CHANGE);
  pinMode(pinAL,INPUT); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinBL, INPUT); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(pinAL),updateEncoderL,CHANGE);

  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test");

  motor_shield.begin();  // create with the default frequency 1.6KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotorR->setSpeed(150);
  myMotorR->run(FORWARD);
  // turn on motor
  myMotorR->run(RELEASE);

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotorL->setSpeed(150);
  myMotorL->run(FORWARD);
  // turn on motor
  myMotorL->run(RELEASE);
}

void loop() {
  uint8_t i;

  Serial.println(F("FORWARD"));
  commandDir = 'F';

  if(runRight){myMotorR->run(FORWARD);}
  if(runLeft){myMotorL->run(FORWARD);}
  for (i=0; i<maxMotorSpeed; i++) {
    if(runRight){myMotorR->setSpeed(i);}
    if(runLeft){myMotorL->setSpeed(i);}
    delay(speedUpdateTime);
  }
  for (i=maxMotorSpeed; i!=0; i--) {
    if(runRight){myMotorR->setSpeed(i);}
    if(runLeft){myMotorL->setSpeed(i);}
    delay(speedUpdateTime);
  }
  delay(1000);

  Serial.println(F("BACK"));
  commandDir = 'B';

  if(runRight){myMotorR->run(BACKWARD);}
  if(runLeft){myMotorL->run(BACKWARD);}
  for (i=0; i<maxMotorSpeed; i++) {
    if(runRight){myMotorR->setSpeed(i);}
    if(runLeft){myMotorL->setSpeed(i);}
    delay(speedUpdateTime);
  }
  for (i=maxMotorSpeed; i!=0; i--) {
    if(runRight){myMotorR->setSpeed(i);}
    if(runLeft){myMotorL->setSpeed(i);}
    delay(speedUpdateTime);
  }

  if(runRight){myMotorR->run(RELEASE);}
  if(runLeft){myMotorL->run(RELEASE);}
  delay(printLeft);
}

void updateEncoderR(){
  // Encoder test on 29 Aug 2021: this is the correct direction
  if(digitalRead(pinAR) == digitalRead(pinBR)){
    encCounterR++;
    encDirR = 'F';
  }
  else{
    encCounterR--;
    encDirR = 'B';
  }


  if(printRight&&(oldEncCounterR != encCounterR)) {
    Serial.print("R CDir: ");
    Serial.print(commandDir);
    Serial.print(", EDir: ");
    Serial.print(encDirR);
    Serial.print(" | C: ");
    Serial.println(encCounterR);
    oldEncCounterR = encCounterR;
  }
}

void updateEncoderL(){
  if(digitalRead(pinAL) != digitalRead(pinBL)){
    encCounterL++;
    encDirL = 'F';
  }
  else{
    encCounterL--;
    encDirL = 'B';
  }

  if(printLeft&&(oldEncCounterL != encCounterL)) {
    Serial.print(F("L CDir: "));
    Serial.print(commandDir);
    Serial.print(F(", EDir: "));
    Serial.print(encDirL);
    Serial.print(F(" | Count: "));
    Serial.println(encCounterL);
    oldEncCounterL = encCounterL;
  }
}
