# 1 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/test_gpio_lasers.ino"
//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

# 11 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/test_gpio_lasers.ino" 2
# 12 "/home/lloydf/Arduino/PB3D/tests/test_gpio_lasers/test_gpio_lasers.ino" 2

LaserManager _laser_manager = LaserManager();

//---------------------------------------------------------------------------
// SETUP
void setup(){
    Serial.begin(115200);
    // Only use below to stop start up until USB cable connected
    while(!Serial){}

    Wire.begin(); // Join I2C bus as leader
    delay(500); // Needed to ensure sensors work, delay allows sub-processors to start up

    // SERIAL: POST SETUP
    Serial.println();
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(("PB3D: LASER GPIO ACTIVATION TEST"))));
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(("--------------------------------"))));

    //----------------------------------------------------------------------------
    // LASER MANAGER
    _laser_manager.begin();

    // Final setup - increase I2C clock speed
    Wire.setClock(400000); // 400KHz
}

//---------------------------------------------------------------------------
// LOOP
void loop(){
    _laser_manager.update();
}
