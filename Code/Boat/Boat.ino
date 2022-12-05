
#include <RadioLib.h>
#include <SPI.h>
#include <Servo.h>
#include <Ultrasonicsensor.h>

#include "generalDefines.h"

// Arduino pins
// HC12 Radio-module pins
//#define HC12TX 6
//#define HC12RX 5

// Servo pinsP
#define SERVOPIN 2
#define WATERPISTOLPIN 6
#define MOTORPIN 5

#define RFM69_RST 9
#define RFM69_CS 10
#define RFM69_INT 3

#define TRIGGERPIN 7
#define ECHOPIN 8

#define MYNODEID 3   // My node ID
#define MIN_DISTANCE 500 //500mm

// prototypes
void steer(uint8_t deg);
void motorPWM(int8_t dutyCycle);

// SoftwareSerial HC12(HC12TX, HC12RX); // Define HC12 communication pins
Servo steeringServo;  // Define servo name
Servo waterPistol;

RFM69 Radio(RFM69_CS, RFM69_INT);

UltrasonicSensorArray DistanceSensors(TRIGGERPIN, ECHOPIN, 3);

bool slowMode = false;
uint16_t distances[NUMBER_OF_SENSORS];

void setup() {
    Serial.begin(9600);

    pinMode(MOTORPIN, OUTPUT);
    pinMode(TRIGGERPIN, OUTPUT);
    pinMode(ECHOPIN, INPUT);
    pinMode(RFM69_RST, OUTPUT);

    digitalWrite(MOTORPIN, LOW);
    digitalWrite(RFM69_RST, LOW);

    // Servo-steer
    waterPistol.attach(WATERPISTOLPIN);
    steeringServo.attach(SERVOPIN);
    steeringServo.write(90);

    // Serial.println("RFM69 TX Test!");
    // Serial.println();

    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    while (!Radio.initialize(RF69_915MHZ, MYNODEID, NETWORKID)) {
        Serial.println("RFM69 radio init failed");
    }
    Radio.setHighPower();
    Radio.setIsrCallback(myCallback);
}

void loop() {
    if (millis() - Radio.timer > 3000) {
        motorPWM(100);
    }

    DistanceSensors.getSensorDistancemm(distances);

    if (!checkDistance(distances, MIN_DISTANCE)) {
        slowMode = true;
    }
}

void myCallback() {
    Serial.write("Callback called");
    motorPWM(Radio.DATA[0] - 100);
    if (Radio.DATA[1] > 100) {
        waterPistol.write(40);
    } else {
        steer(Radio.DATA[1]);
        waterPistol.write(0);
    }
    Radio.sendFrame(3, distances, 3);
    Radio.setMode(RF69_MODE_RX);
}

/**
 *  in Degrees°, 0 - 90
 */
void steer(uint8_t deg) {
    if (deg > 90) {
        deg = 90;
    }
    steeringServo.write(45 + deg);
}

/**
 *    0% to 100%
 */
void motorPWM(int8_t dutyCycle) {
    if (dutyCycle > 200) {
        dutyCycle = 200;
    }
    if (dutyCycle < 100) {
        dutyCycle = 100;
    }
    dutyCycle = dutyCycle - 100;
    analogWrite(MOTORPIN, dutyCycle * 2.55);
}

bool checkMinDistance(uint16_t* lengths, uint16_t minLength){
    for(uint8_t i = 0; i< NUMBER_OF_SENSORS; i++){
        if(lengths[i]<minLength){
            return false;
        }
    }
    return true;
}