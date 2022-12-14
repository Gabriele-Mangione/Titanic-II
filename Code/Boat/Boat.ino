
#include <RadioLib.h>
#include <SPI.h>
#include <Servo.h>
#include <Ultrasonicsensor.h>

#include "generalDefines.h"


#define SERVOPIN 2
#define WATERPISTOLPIN 6
#define MOTORPIN 5

//Radio module pins
#define RFM69_RST 9
#define RFM69_CS 10
#define RFM69_INT 3

//Radio IDs
#define MYNODEID 3   // My node ID

//Ultrasound sensor pins
#define TRIGGERPIN 7
#define ECHOPIN 8

#define MIN_DISTANCE 500

#ifndef __XMEGA__
#define UCSR0A USART1_STATUS
#define UDR0 USART1_RXDATAL
#define FE0 USART_FERR_bp
#define USART_RX_vect USART1_RXC_vect
#endif

// prototypes
void steer(uint8_t deg);
void motorPWM(int8_t dutyCycle);

Servo SteeringServo;
Servo WaterPistol;

RFM69 Radio(RFM69_CS, RFM69_INT);

UltrasonicSensorArray DistanceSensors(TRIGGERPIN, ECHOPIN, 3);

bool slowMode = false;
uint16_t distances[NUMBER_OF_SENSORS];

void setup() {
  //for testing
  Serial.begin(9600);

  //set pin modes
  pinMode(MOTORPIN, OUTPUT);
  pinMode(TRIGGERPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(RFM69_RST, OUTPUT);
  pinMode(RFM69_INT, INPUT_PULLUP);

    digitalWrite(MOTORPIN, LOW);
    digitalWrite(RFM69_RST, LOW);

  //Assign servo pins to servo objects
  //WaterPistol.attach(WATERPISTOLPIN, 0, 40);
  SteeringServo.attach(SERVOPIN);
  SteeringServo.write(90);
  //WaterPistol.write(0);

  //Radio init
  // manual Radio reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

    while (!Radio.initialize(RF69_915MHZ, MYNODEID, NETWORKID)) {
        Serial.println("RFM69 radio init failed");
    }
    Radio.setHighPower();

    Radio.setIsrCallback(myCallback);
    Radio.setIrq(RFM69_INT);
    Radio.setMode(RF69_MODE_STANDBY);
    Radio.receiveDone();
}
/*
void myInterruptFunction(){
        interruptHandler();

    if (_mode == RF69_MODE_RX && PAYLOADLEN > 0)
    {
        setMode(RF69_MODE_STANDBY); // enables interrupts
        return true;
    }
    else if (_mode == RF69_MODE_RX) // already in RX no payload yet
    {
        return false;
    }
    receiveBegin();
    return false;
}*/

void loop() {
    if (millis() - Radio.timer > 3000) {
        motorPWM(100);
    }

    DistanceSensors.getSensorDistance(distances);

    if (!checkMinDistance(distances,NUMBER_OF_SENSORS, MIN_DISTANCE)) {
        slowMode = true;
    }
    Serial.println(SteeringServo.read());
}

void myCallback() {
    //Serial.write("Callback called");
    motorPWM(Radio.DATA[0] - 100);
    if (Radio.DATA[1] > 100) {
        WaterPistol.write(40);
    } else {
        steer(Radio.DATA[1]);
        WaterPistol.write(0);
    }
    //Radio.sendFrame(3, distances, 3);
}

/**
 *  in Degrees°, 0 - 90
 */
/**
 * @brief steer in angle "deg"
 * 
 * @param deg degrees 0-90, where 45 is forward
*/
void steer(uint8_t deg) {
  if (deg > 90) {
    deg = 90;
  }
  SteeringServo.write(45 + deg);
}

/**
 * @brief set speed value for motor using PWM
 * 
 * @param dutyCycle 0-200, where 0 is full backwards power, 100 is no movement and 200 is full forwards power
 */
void motorPWM(int8_t dutyCycle) {
    if (dutyCycle > 200) {
        dutyCycle = 200;
    }
    if (dutyCycle < 100) {
        dutyCycle = 100;
    }
    dutyCycle = dutyCycle - 100;
    analogWrite(MOTORPIN, dutyCycle * 2.54);
}
/**
 * @brief check for a certain time if the Radio received a new message
 * 
 * @param stoptime time in ms of duration 
 * @return true when a message was received
 * @return false when stoptime has passed and no message was received
 *
bool timeoutReceiver(unsigned long stoptime) {
  unsigned long time = millis();
  while (millis() - time < stoptime) {
    if (Radio.receiveDone()) {
      return true;
    }
    return true;
}
*/

bool checkMinDistance(uint16_t *distances,uint8_t numOfDistances, uint8_t minDistance){
  for(uint8_t i = 0; i> numOfDistances; i++){
    if(distances[i]<minDistance){
      return true;
    }
  }
  return false;
};