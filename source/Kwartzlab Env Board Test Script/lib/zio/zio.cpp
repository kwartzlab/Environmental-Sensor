#include "zio.h"

bool ZIO::begin(uint8_t address, TwoWire &wirePort) {
  _i2cPort = &wirePort;
  _address = address;

  uint8_t triesBeforeGiveup = 5;

  for (uint8_t x = 0; x < triesBeforeGiveup; x++) {
    _i2cPort->beginTransmission(_address);
    if (_i2cPort->endTransmission() == 0) {
      return true;
    }
    delay(100);
  }
  return false;
}

bool ZIO::ledOn() {
  _i2cPort->beginTransmission(_address);
  _i2cPort->write(COMMAND_LED_ON);
  if (_i2cPort->endTransmission() != 0)
    return false;
  else
    return true;
}

bool ZIO::ledOff() {
  _i2cPort->beginTransmission(_address);
  _i2cPort->write(COMMAND_LED_OFF);
  if (_i2cPort->endTransmission() != 0)
    return false;
  else
    return true;
}

uint16_t ZIO::getValue() {
  uint16_t ADC_VALUE=0;

  _i2cPort->beginTransmission(_address);
  _i2cPort->write(COMMAND_GET_VALUE);
  _i2cPort->endTransmission();

  _i2cPort->requestFrom(_address, 2);

  while (_i2cPort->available()) {
    uint8_t ADC_VALUE_L = _i2cPort->read();
    uint8_t ADC_VALUE_H = _i2cPort->read();
    ADC_VALUE=ADC_VALUE_H;
    ADC_VALUE<<=8;
    ADC_VALUE|=ADC_VALUE_L;
  }
  uint16_t x = _i2cPort->read();
  return ADC_VALUE;
}
