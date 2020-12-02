# Self balancing robot with ESP32

## Hardware 

- LOLIN32 Board
- Pololu DRV8833 dual motor driver
- MPU6050
- Pololu micro geared dc motors (high power 6V, experimenting with various gearbox ratios)

## Software

- Build&IDE: Platformio 
- ESP32 Arduino Core
- For libraries see platformio.ini
- Currently also uses RemoteXY library and app for controll. Just download the RemoteXY library and place it in the lib folder.

## Setup and build

- Download and unpack RemoteXY library into `lib`
- Add src/keys.h with your wifi credentials (if OTA should be supported)
- i2cdevlib needs to be patched to compile for Esp32, see https://github.com/josefadamcik/i2cdevlib/commit/788ecc1614a189ab0fc894ae7d2c7144742922c0 . 
keys.h

```
#define MYSSID "ssid"
#define MYPASS "pass"
```

## Licence

- MIT
