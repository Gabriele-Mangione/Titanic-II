
#include <RFM69.h>
#include <SPI.h>
#include <Servo.h>
#include <Ultrasonicsensor.h>

// Arduino pins
// HC12 Radio-module pins
//#define HC12TX 6
//#define HC12RX 5

// Servo pinsP
#define SERVOPIN 5
#define MOTORPIN 2

#define RFM69_RST 9
#define RFM69_CS 10
#define RFM69_INT 3

#define TRIGGERPIN 7
#define ECHOPIN 8

#define NETWORKID 0  // Must be the same for all nodes
#define MYNODEID 3   // My node ID

// prototypes
void steer(uint8_t deg);
void motorPWM(int8_t dutyCycle);

// SoftwareSerial HC12(HC12TX, HC12RX); // Define HC12 communication pins
Servo steeringServo;  // Define servo name

RFM69 radio(RFM69_CS, RFM69_INT);

UltrasonicSensorArray DistanceSensors(TRIGGERPIN, ECHOPIN, 3);
uint8_t buf[3];
uint16_t distances[3];

void setup() {
    Serial.begin(9600);

    pinMode(MOTORPIN, OUTPUT);
    pinMode(TRIGGERPIN, OUTPUT);
    pinMode(ECHOPIN, INPUT);
    pinMode(RFM69_RST, OUTPUT);

    digitalWrite(MOTORPIN, LOW);
    digitalWrite(RFM69_RST, LOW);

    // Servo-steer
    steeringServo.attach(SERVOPIN);
    steeringServo.write(90);

    // Serial.println("RFM69 TX Test!");
    // Serial.println();

    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    while (!radio.initialize(RF69_915MHZ, MYNODEID, NETWORKID)) {
        Serial.println("RFM69 radio init failed");
    }
    radio.setHighPower();
}

void loop() {
    uint8_t inputMotorSpeed = 100;
    uint8_t inputSteerAngle = 45;

    DistanceSensors.getSensorDistance(distances);
    /*Serial.print("0: ");
    Serial.println(distances[0]);
    Serial.print("1: ");
    Serial.println(distances[1]);
    Serial.print("2: ");
    Serial.println(distances[2]);*/

    buf[0] = distances[0] / 100;
    buf[1] = distances[1] / 100;
    buf[2] = distances[2] / 100;

    if (timeoutReceiver(200)) {
        radio.sendWithRetry(4, buf, 3, 2, 15);
        inputMotorSpeed = radio.DATA[0];
        inputSteerAngle = radio.DATA[1];
    } else {
        // Serial.print("error-Timeout");
    }
    motorPWM(inputMotorSpeed);
    steer(inputSteerAngle);
}

/**
 *  in Degrees°, 0 - 60
 *  steer from 60° to 120°
 */
void steer(uint8_t deg) {
    // Serial.print("steer = ");
    if (deg > 90) {
        deg = 90;
    }
    // Serial.println(45 + deg);
    steeringServo.write(45 + deg);
}

/**
 *    0% to 100%
 */
void motorPWM(int8_t dutyCycle) {
    dutyCycle = dutyCycle - 100;
    if (dutyCycle > 100) {
        dutyCycle = 100;
    }
    if (dutyCycle < 0) {
        dutyCycle = 0;
    }
    if (distances[0] < 100 || distances[1] < 100 || distances[2] < 100) {
        dutyCycle /= 10;
    }
    // Serial.print("PWM = ");
    // Serial.print(dutyCycle);
    analogWrite(MOTORPIN, dutyCycle * 10.24);
}

bool timeoutReceiver(unsigned long stoptime) {
    unsigned long time = millis();
    while (millis() - time < stoptime) {
        if (radio.receiveDone()) {
            return true;
        }
        // yield();
    }
    /*
    for(uint16_t i = 0; i<timestotry;i++){
      if (radio.receiveDone()) {
        return true;
      }
      yield();
    }*/
    return false;
}