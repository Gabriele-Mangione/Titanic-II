
#include "Ultrasonicsensor.h"

UltrasonicSensor::UltrasonicSensor(uint8_t TriggerPin, uint8_t EchoPin){
    triggerPin = TriggerPin;
    echoPin = EchoPin;
}

byte UltrasonicSensor::getSensorDistance() {
  digitalWrite(triggerPin, LOW);                 // Clears the trigPin condition
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);                      // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(triggerPin, LOW);
  return (pulseIn(echoPin, HIGH) * 0.034 / 2);        // Calculating the distance Speed of sound wave divided by 2 (go and back)
}

UltrasonicSensorArray::UltrasonicSensorArray(uint8_t TriggerPin, uint8_t EchoPin, uint8_t NumberOfSensors):UltrasonicSensor(TriggerPin,EchoPin){
  triggerPin = TriggerPin;
  echoPin = EchoPin;
  numberOfSensors = NumberOfSensors;
  pinMode(triggerPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
}

void UltrasonicSensorArray::getSensorDistance(uint16_t* distance){
  digitalWrite(triggerPin, LOW);              // Clears the trigPin condition
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);                      // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(triggerPin, LOW);
  for(byte i = 0; i < numberOfSensors; i++){
    distance[i] = pulseIn(echoPin, HIGH)* 0.343 /2;
  }
}