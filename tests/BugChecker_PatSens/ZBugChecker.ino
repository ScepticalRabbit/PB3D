#include "PatSensor.h"

PatSensor patSensorObj = PatSensor();

void setup() {
  patSensorObj.begin();
}

void loop() {
  patSensorObj.update();
}
