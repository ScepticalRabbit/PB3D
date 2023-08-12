#include "LF_Seeed_vl53l0x.h"
Seeed_vl53l0x VL53L0X;

#define NEW_I2C_ADDR 0x31

void setup() {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    Serial.begin(115200);
    
    Status = VL53L0X.VL53L0X_common_init(NEW_I2C_ADDR);
    delay(5000);
    
    if (VL53L0X_ERROR_NONE != Status) {
        Serial.println("start vl53l0x measurement 1 failed!");
        VL53L0X.print_pal_error(Status);
        while (1);
    }
    
    VL53L0X.VL53L0X_continuous_ranging_init();
    if (VL53L0X_ERROR_NONE != Status) {
        Serial.println("start vl53l0x measurement 2 failed!");
        VL53L0X.print_pal_error(Status);
        while (1);
    }
}

void loop() {
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    VL53L0X.PerformContinuousRangingMeasurement(&RangingMeasurementData);
    
    if (RangingMeasurementData.RangeMilliMeter >= 2000) {
        Serial.println("out of range");
    } 
    else{
        Serial.print("distance: ");
        Serial.println(RangingMeasurementData.RangeMilliMeter);
    }
    
    delay(100);
}
