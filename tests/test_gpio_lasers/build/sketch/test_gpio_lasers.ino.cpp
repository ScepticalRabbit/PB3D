#line 1 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/test_gpio_lasers.ino"
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

LaserManager _laser_manager = LaserManager();

//---------------------------------------------------------------------------
// SETUP
void setup(){
  // Start the serial
  Serial.begin(115200);
  // Only use below to stop start up until USB cable connected
  while(!Serial){}

  // Initialize I2C communications for sensors and sub boards
  Wire.begin();  // Join I2C bus as leader
  delay(500);   // Needed to ensure sensors work, delay allows sub-processors to start up

  // SERIAL: POST SETUP
  Serial.println();
  Serial.println(F("PB3D: LASER GPIO ACTIVATION TEST"));
  Serial.println(F("--------------------------------"));

  //----------------------------------------------------------------------------
  // LASER MANAGER
  _laser_manager.begin();
  //----------------------------------------------------------------------------

  // Final setup - increase I2C clock speed
  Wire.setClock(400000); // 400KHz

}

//---------------------------------------------------------------------------
// LOOP
void loop(){
  _laser_manager.update();
}


