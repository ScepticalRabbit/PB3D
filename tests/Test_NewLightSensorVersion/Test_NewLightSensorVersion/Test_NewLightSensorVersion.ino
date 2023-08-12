#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Adafruit VEML7700 No Wait Test");

  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1);
  }
  Serial.println("Sensor found");

  // == OPTIONAL =====
  // Can set non-default gain and integration time to
  // adjust for different lighting conditions.
  // =================
  // veml.setGain(VEML7700_GAIN_1_8);
  // veml.setIntegrationTime(VEML7700_IT_100MS);

  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }

  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }
}

void loop() {
  unsigned long startTime;

  Serial.println("-------------------------------------------");
  
  startTime = millis();
  Serial.print("Raw ALS without wait: "); Serial.print(veml.readALS());
  Serial.print("  ["); Serial.print(millis() - startTime); Serial.println("ms]");

  startTime = millis();
  Serial.print("Raw ALS with wait: "); Serial.print(veml.readALS(true));
  Serial.print("  ["); Serial.print(millis() - startTime); Serial.println("ms]");

  startTime = millis();
  Serial.print("LUX without wait: "); Serial.print(veml.readLux(VEML_LUX_NORMAL_NOWAIT));
  Serial.print("  ["); Serial.print(millis() - startTime); Serial.println("ms]");

  startTime = millis();
  Serial.print("LUX with wait: "); Serial.print(veml.readLux());
  Serial.print("  ["); Serial.print(millis() - startTime); Serial.println("ms]");

  delay(2000);
}
