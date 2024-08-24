#include "PatSensor.h"

PatSensor pat_sensor = PatSensor();

void setup() {
  pat_sensor.begin();
}

void loop() {
  pat_sensor.update();
}
