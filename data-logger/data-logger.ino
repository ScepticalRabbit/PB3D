//==============================================================================
// PB3D: A 3D printed pet robot
//==============================================================================
//
// Author: ScepticalRabbit
// License: MIT
// Copyright (C) 2024 ScepticalRabbit
//------------------------------------------------------------------------------

#include "RFDataLoggerRX.h"
#include "PB3DTimer.h"
#include "PB3DStateData.h"

RFDataLoggerRX rfLogger;

//----------------------------------------------------------------------------
// SETUP
void setup(){
  Serial.begin(115200);
  while(!Serial){delay(10);}
  
  rfLogger.begin();
  delay(1000);
}

//----------------------------------------------------------------------------
// LOOP
void loop(){
  rfLogger.update();
}
//----------------------------------------------------------------------------
