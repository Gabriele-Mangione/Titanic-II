
#ifndef _ULTRASONICSENSOR_
#define _ULTRASONICSENSOR_

#include "Arduino.h"

class UltrasonicSensor{
    public:
    UltrasonicSensor(uint8_t TriggerPin, uint8_t EchoPin);
    byte getSensorDistance();

    uint8_t triggerPin = 0;
    uint8_t echoPin = 0;

};

class UltrasonicSensorArray : public UltrasonicSensor{
    public:
    UltrasonicSensorArray(uint8_t TriggerPin, uint8_t EchoPin, uint8_t NumberOfSensors);
    void getSensorDistance(uint16_t* distance);

    uint8_t numberOfSensors = 0;
};
#endif