
#ifndef _softtwi_h_
#define _softtwi_h_

#include <Arduino.h>

#define BUFFER_LENGTH 32

class SoftTWI {
public:
  void attach(int sdaPin, int sclPin);
  void begin();
  void beginTransmission(uint8_t address);
  void beginTransmission(int address);
  void endTransmission();
  uint8_t write(uint8_t val);
  uint8_t requestFrom(uint8_t address, uint8_t quantity);
  uint8_t read(){return receive();};
  uint8_t receive();
  int16_t receiveInt16();

private:
  uint8_t _sdaBit;
  uint8_t _sdaPort;
  volatile uint8_t* _sdaMode;
  volatile uint8_t* _sdaOut;
  volatile uint8_t* _sdaIn;

  uint8_t _sclBit;
  uint8_t _sclPort;
  volatile uint8_t* _sclMode;
  volatile uint8_t* _sclOut;
  volatile uint8_t* _sclIn;

  uint8_t* _rxBuffer;
  uint8_t _rxBufferIndex;
  uint8_t _rxBufferLength;

  uint8_t readFrom(uint8_t address, uint8_t* buffer, uint8_t quantity);
  uint8_t read(uint8_t more);
};

extern SoftTWI SoftWire;

#endif


