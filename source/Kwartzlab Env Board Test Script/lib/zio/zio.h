#ifndef ZIO_H_
#define ZIO_H_

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>

#define ZIO_DEFAULT_ADDRESS 0x38

#define COMMAND_LED_OFF 0x00
#define COMMAND_LED_ON 0x01
#define COMMAND_GET_VALUE 0x05
#define COMMAND_NOTHING_NEW 0x99

class ZIO {
private:
    TwoWire *_i2cPort = NULL;
    uint8_t _address = NULL;

public:
    bool begin(uint8_t address, TwoWire &wirePort);
    bool ledOn();
    bool ledOff();
    uint16_t getValue();
};

#endif
