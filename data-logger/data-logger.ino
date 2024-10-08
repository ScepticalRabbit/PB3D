//==============================================================================
// PB3D: A pet robot that is 3D printed
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "RFDataLoggerRX.h"
#include "PB3DTimer.h"

RFDataLoggerRX rfLogger;

//----------------------------------------------------------------------------
// SETUP
void setup(){
  Serial.begin(115200);
  //while(!Serial){delay(10);}
  rfLogger.begin();
  delay(1000);
}

//----------------------------------------------------------------------------
// MAIN LOOP
void loop(){
  rfLogger.update();
}
//----------------------------------------------------------------------------
