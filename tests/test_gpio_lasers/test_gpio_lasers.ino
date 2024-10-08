//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include <Arduino.h>
#include "LaserManager.h"

LaserManager _laser_manager = LaserManager();

//---------------------------------------------------------------------------
// SETUP
void setup(){
    Serial.begin(115200);
    // Only use below to stop start up until USB cable connected
    while(!Serial){}

    Wire.begin();  // Join I2C bus as leader
    delay(500);   // Needed to ensure sensors work, delay allows sub-processors to start up

    // SERIAL: POST SETUP
    Serial.println();
    Serial.println(F("PB3D: LASER GPIO ACTIVATION TEST"));
    Serial.println(F("--------------------------------"));

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

