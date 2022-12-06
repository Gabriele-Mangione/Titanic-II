
#ifndef MY_RADIO
#define MY_RADIO
#include "Arduino.h"
#include "SPI.h"

class MyRadio{
    public:
    MyRadio(uint8_t CSPin, uint8_t INTPin, SPIClass *spi);
    bool initialise(uint8_t freqBand, uint16_t ID, uint8_t networkID = 1);
    bool sendData(uint16_t toAdress, const void* buffer, uint8_t bufferSize);
    void setInterruptCallback(void (*callback)());
    protected:
    uint8_t _CSPin;
    uint8_t _INTPin;
    SPIClass *_spi = NULL;
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t val);
    SPISettings _spiSettings;

};


#endif