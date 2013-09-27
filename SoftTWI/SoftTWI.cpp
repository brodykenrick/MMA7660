
//#include <WProgram.h>
//#include <pins_arduino.h>
#include <Arduino.h>

#include "SoftTWI.h"

#define TWI_DELAY_T2 5
#define twicbi(reg, bit) *reg &= ~bit
#define twisbi(reg, bit) *reg |= bit
#define twild(reg, bit) (*reg & bit)

void SoftTWI::attach(int sdaPin, int sclPin) {
  _sdaBit = digitalPinToBitMask(sdaPin);
  _sdaPort = digitalPinToPort(sdaPin);
  if (_sdaPort == NOT_A_PIN) return;
  _sdaMode = portModeRegister(_sdaPort);
  _sdaOut = portOutputRegister(_sdaPort);
  _sdaIn = portInputRegister(_sdaPort);

  _sclBit = digitalPinToBitMask(sclPin);
  _sclPort = digitalPinToPort(sclPin);
  if (_sclPort == NOT_A_PIN) return;
  _sclMode = portModeRegister(_sclPort);  
  _sclOut = portOutputRegister(_sclPort);
  _sclIn = portInputRegister(_sclPort);
}

void SoftTWI::begin() {
  // init buffer for reads
  _rxBuffer = (uint8_t*) calloc(BUFFER_LENGTH, sizeof(uint8_t));
  _rxBufferIndex = 0;
  _rxBufferLength = 0;

  twicbi(_sdaMode, _sdaBit); 
  twicbi(_sclMode, _sclBit); 
  twicbi(_sdaOut, _sdaBit); 
  twicbi(_sclOut, _sclBit); 
}

void SoftTWI::beginTransmission(uint8_t address) {
  twisbi(_sdaMode, _sdaBit); // force SDA low
  delayMicroseconds(TWI_DELAY_T2);

  write(address); // write address
}

void SoftTWI::beginTransmission(int address) {
  beginTransmission((uint8_t)address);
}

void SoftTWI::endTransmission() {
  twisbi(_sclMode, _sclBit); // force SCL low
  twisbi(_sdaMode, _sdaBit); // force SDA low
  delayMicroseconds(TWI_DELAY_T2);
  twicbi(_sclMode, _sclBit); // release SCL
  delayMicroseconds(TWI_DELAY_T2);
  twicbi(_sdaMode, _sdaBit); // release SDA
  delayMicroseconds(TWI_DELAY_T2);
}

uint8_t SoftTWI::write(uint8_t val /* r24 */) {
  for(uint8_t b=0; b<8; b++) {
    twisbi(_sclMode, _sclBit); // force SCL low
    if(val & 0x80) { // if msb set
      twicbi(_sdaMode, _sdaBit); // release SDA
    }
    else{
      twisbi(_sdaMode, _sdaBit); // force SDA low
    }
    delayMicroseconds(TWI_DELAY_T2); 
    twicbi(_sclMode, _sclBit); // release SCL
    delayMicroseconds(TWI_DELAY_T2);

    val = val << 1;
  }

  // ack
  twisbi(_sclMode, _sclBit); // force SCL low
  twicbi(_sdaMode, _sdaBit); // release SDA
  delayMicroseconds(TWI_DELAY_T2); 
  twicbi(_sclMode, _sclBit); // release SCL
  while(!twild(_sclIn, _sclBit)); // wait SCL high (in case wait states are inserted)
  int result = twild(_sdaIn, _sdaBit);

  delayMicroseconds(TWI_DELAY_T2);
  return result;
}

uint8_t SoftTWI::requestFrom(uint8_t address, uint8_t quantity) {
  // clamp to buffer length
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }

  // perform blocking read into buffer
  uint8_t read = readFrom(address, _rxBuffer, quantity);

  // set rx buffer iterator vars
  _rxBufferIndex = 0;
  _rxBufferLength = read;

  return read;
}

uint8_t SoftTWI::receive() {
  // default to returning null char
  // for people using with char strings
  uint8_t value = '\0';

  // get each successive byte on each call
  if(_rxBufferIndex < _rxBufferLength){
    value = _rxBuffer[_rxBufferIndex];
    ++_rxBufferIndex;
  }

  return value;
}

int16_t SoftTWI::receiveInt16() {
  byte msb = receive();
  byte lsb = receive();
  return( (msb<<8) | lsb);
}

uint8_t SoftTWI::readFrom(uint8_t address, uint8_t* buffer, uint8_t quantity) {
  int i;
  beginTransmission(address);
  for(i = 0; i < quantity; i++) {
    buffer[i] = read(i < (quantity - 1)); // ask for more until last byte
  }
  endTransmission();
  return i;
}

uint8_t SoftTWI::read(uint8_t more) {
  uint8_t result = 0;

  twisbi(_sclMode, _sclBit); // force SCL low
  twicbi(_sdaMode, _sdaBit); // release SDA
  for(uint8_t b = 0; ; b++){
    twisbi(_sclMode, _sclBit); // force SCL low
    delayMicroseconds(TWI_DELAY_T2);

    twicbi(_sclMode, _sclBit); // release SCL
    delayMicroseconds(TWI_DELAY_T2);
    while(!twild(_sclIn, _sclBit)); // loop until SCL is high (allow slave to stretch SCL)

    result |= twild(_sdaIn, _sdaBit) ? 0x01 : 0x00;
    if(b == 7) break; // don't rotate last bit
    result = result << 1;
  }

  // send ack
  twisbi(_sclMode, _sclBit); // force SCL low
  if(more){
    twisbi(_sdaMode, _sdaBit); // force SDA low
  }
  else{
    twicbi(_sdaMode, _sdaBit); // release SDA
  }
  delayMicroseconds(TWI_DELAY_T2);
  twicbi(_sclMode, _sclBit); // release SCL
  while(!twild(_sclIn, _sclBit)); // wait SCL high
  delayMicroseconds(TWI_DELAY_T2);

  return result;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

SoftTWI SoftWire = SoftTWI();

