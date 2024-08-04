# 1 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/test_gpio_lasers.ino"
//---------------------------------------------------------------------------
// PET BOT 3D - PB3D!
// TEST: PCF8574 gpio activate lasers VL53LOx
//---------------------------------------------------------------------------
/*
TODO

Author: Lloyd Fletcher
*/

# 12 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/test_gpio_lasers.ino" 2
# 13 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/test_gpio_lasers.ino" 2

LaserManager _laserManager = LaserManager();

//---------------------------------------------------------------------------
// SETUP
void setup(){
  // Start the serial
  Serial.begin(115200);
  // Only use below to stop start up until USB cable connected
  while(!Serial){}

  // Initialize I2C communications for sensors and sub boards
  Wire.begin(); // Join I2C bus as leader
  delay(500); // Needed to ensure sensors work, delay allows sub-processors to start up

  // SERIAL: POST SETUP
  Serial.println();
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(("PB3D: LASER GPIO ACTIVATION TEST"))));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(("--------------------------------"))));

  //----------------------------------------------------------------------------
  // LASER MANAGER
  _laserManager.begin();
  //----------------------------------------------------------------------------

  // Final setup - increase I2C clock speed
  Wire.setClock(400000); // 400KHz

}

//---------------------------------------------------------------------------
// LOOP
void loop(){
  _laserManager.update();
}

//---------------------------------------------------------------------------
// FUNCTIONS
