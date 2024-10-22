# PB3D Work Plan
-----------------------------------------------------------
PB3D is a 3D printed pet robot based on arduino.

## TODO: Work Plan
-----------------------------------------------------------
- Check power consumption with INA219
- Check radio TX/RX still works for speed
    - Fix statedata to have packets of 60 bytes
- Tune PIDs
    - Speed PIDs
    - Pos PIDs
- Check smooth movement code
    - Tail - tune smoother
    - Wheels - tune smoother

## NOTES:
-----------------------------------------------------------

### Adafruit GPIO exapnder PCF8574
https://learn.adafruit.com/adafruit-pcf8574
https://github.com/adafruit/Adafruit_PCF8574
https://www.adafruit.com/product/5545

If you want to send a GPIO output logic level to some other device or peripheral, the light pull-up acts as high logic out, the strong ground output acts as low logic out.

## Adafruit RFM69 Packet Radio
https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/using-the-rfm69-radio

- Only 60 bytes per send!
- float = 32 bits = 4 bytes
- int16_t = 16 bits = 2 bytes
- int8_t = 8 bites = 1 bytes

## I2C Addresses
// #define FXOS8700_ADDRESS (0x1F) // 0011111
// #define FXAS21002C_ADDRESS (0x21) // 0100001


