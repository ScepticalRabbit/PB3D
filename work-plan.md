# PB3D Work Plan
-----------------------------------------------------------
PB3D is a 3D printed pet robot based on arduino.

## Code Work Plan
-----------------------------------------------------------

### Collision Avoidance
- Debug and test new collision avoidance algorithms
    - Seems to be ok with new classes now
- Test lasers under different conditions
- Test lasers for accurate ranging time

### Movement
- Add smooth accelerate to speed and coast to stop

## CAD Work Plan
-----------------------------------------------------------

### Version 4 Updates
- Ball Castor: Larger ball castor?
- Laser Cones: make cone wider

- DONE - Wheels: 3D printed wheel hubs, need hub D shaft interface test for tolerances

### Version 4 Fixes
- DONE - Laser Cones: clearance for on/off pin
- DONE - Bumper: make thicker
- DONE - Motors: check clearance for new motors

## 3DP and Assembly Plan
-----------------------------------------------------------
- 

## NOTES:
-----------------------------------------------------------

### Adafruit GPIO exapnder PCF8574
https://learn.adafruit.com/adafruit-pcf8574
https://github.com/adafruit/Adafruit_PCF8574
https://www.adafruit.com/product/5545

If you want to send a GPIO output logic level to some other device or peripheral, the light pull-up acts as high logic out, the strong ground output acts as low logic out.

## I2C Addresses
// #define FXOS8700_ADDRESS (0x1F) // 0011111
// #define FXAS21002C_ADDRESS (0x21) // 0100001


