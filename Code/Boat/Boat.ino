
#include <RFM69.h>
#include <SPI.h>
#include <Servo.h>
#include <Ultrasonicsensor.h>

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

#define NETWORKID 0  // Must be the same for all nodes
#define MYNODEID 3   // My node ID
#define MAX_DISTANCE 500

// prototypes
void steer(uint8_t deg);
void motorPWM(int8_t dutyCycle);
bool timeoutReceiver(unsigned long stoptime);

// SoftwareSerial HC12(HC12TX, HC12RX); // Define HC12 communication pins
Servo steeringServo;  // Define servo name
Servo waterPistol;

RFM69 radio(RFM69_CS, RFM69_INT);

UltrasonicSensorArray DistanceSensors(TRIGGERPIN, ECHOPIN, 3);

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

  while (!radio.initialize(RF69_915MHZ, MYNODEID, NETWORKID)) {
    Serial.println("RFM69 radio init failed");
  }
  radio.setHighPower();
}

void loop() {
  uint8_t inputMotorSpeed = 100;
  uint8_t inputSteerAngle = 45;

  uint16_t distances[3];
  DistanceSensors.getSensorDistance(distances);

  if (distances[0] < MAX_DISTANCE || distances[1] < MAX_DISTANCE || distances[2] < MAX_DISTANCE) {
    inputMotorSpeed /= 5;
  }

  distances[0] /= 100;
  distances[1] /= 100;
  distances[2] /= 100;

  if (timeoutReceiver(1000)) {
    radio.sendWithRetry(4, distances, 3, 2, 20);
    inputMotorSpeed = radio.DATA[0];
    inputSteerAngle = radio.DATA[1];
  }
  else {
    Serial.print("error-Timeout");
  }
  motorPWM(inputMotorSpeed);
  if (inputSteerAngle > 100) {
    waterPistol.write(40);
  }
  else {
    steer(inputSteerAngle);
    waterPistol.write(0);
  }
}

/**
 *  in Degrees°, 0 - 60
 *  steer from 60° to 120°
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

bool timeoutReceiver(unsigned long stoptime) {
  unsigned long time = millis();
  while (millis() - time < stoptime) {
    if (radio.receiveDone()) {
      return true;
    }
    // yield();
  }
  return false;
}