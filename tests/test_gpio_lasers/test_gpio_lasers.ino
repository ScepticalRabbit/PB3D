//---------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// TEST: PCF8574 gpio activate lasers VL53LOx
//---------------------------------------------------------------------------
/*
TODO

Author: Lloyd Fletcher
*/

#include <Arduino.h>
#include "LaserManager.h"
#include "LaserSensor.h"

LaserManager _laserManager = LaserManager();
/*
LaserSensor _laserUC = LaserSensor(ADDR_LSR_UC,'U');
LaserSensor _laserDL = LaserSensor(ADDR_LSR_DL,'H');
LaserSensor _laserDR = LaserSensor(ADDR_LSR_DR,'K');
*/
//---------------------------------------------------------------------------
// SETUP
void setup(){
  // Start the serial
  Serial.begin(115200);
  // Only use below to stop start up until USB cable connected
  while(!Serial){}

  // Initialize I2C communications for sensors and sub boards
  Wire.begin();  // Join I2C bus as leader
  delay(1000);   // Needed to ensure sensors work, delay allows sub-processors to start up

  // SERIAL: POST SETUP
  Serial.println();
  Serial.println(F("PB3D: LASER GPIO TEST"));
  Serial.println(F("------------------------------"));

  //----------------------------------------------------------------------------
  // LASER MANAGER
  _laserManager.begin();
  //----------------------------------------------------------------------------

}

//---------------------------------------------------------------------------
// LOOP
void loop(){

}

//---------------------------------------------------------------------------
// FUNCTIONS

