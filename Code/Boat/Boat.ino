
#include <RFM69.h>
#include <SPI.h>
#include <Servo.h>
#include <Ultrasonicsensor.h>


#define SERVOPIN 2
#define WATERPISTOLPIN 6
#define MOTORPIN 5

//Radio module pins
#define RFM69_RST 9
#define RFM69_CS 10
#define RFM69_INT 3

//Radio IDs
#define NETWORKID 0  // Must be the same for all nodes
#define MYNODEID 3   // My node ID

//Ultrasound sensor pins
#define TRIGGERPIN 7
#define ECHOPIN 8

#define MAX_DISTANCE 500


// prototypes
void steer(uint8_t deg);
void motorPWM(int8_t dutyCycle);
bool timeoutReceiver(unsigned long stoptime);

Servo SteeringServo;
Servo WaterPistol;

RFM69 Radio(RFM69_CS, RFM69_INT);

UltrasonicSensorArray DistanceSensors(TRIGGERPIN, ECHOPIN, 3);

void setup() {
  //for testing
  //Serial.begin(9600);

  //set pin modes
  pinMode(MOTORPIN, OUTPUT);
  pinMode(TRIGGERPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(RFM69_RST, OUTPUT);

  digitalWrite(MOTORPIN, LOW);
  digitalWrite(RFM69_RST, LOW);

  //Assign servo pins to servo objects
  WaterPistol.attach(WATERPISTOLPIN);
  SteeringServo.attach(SERVOPIN);
  SteeringServo.write(90);

  // manual Radio reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  //Radio init
  while (!Radio.initialize(RF69_915MHZ, MYNODEID, NETWORKID)) {
    Serial.println("RFM69 Radio init failed");
  }
  Radio.setHighPower();
}

void loop() {
  //standard values, in case no inputs are received from radio
  uint8_t inputMotorSpeed = 100;
  uint8_t inputSteerAngle = 45;

  uint16_t distances[3];
  DistanceSensors.getSensorDistance(distances);

  //if an object within 500mm is detected
  if (distances[0] < MAX_DISTANCE || distances[1] < MAX_DISTANCE || distances[2] < MAX_DISTANCE) {
    inputMotorSpeed /= 5;
  }

  distances[0] /= 100;
  distances[1] /= 100;
  distances[2] /= 100;
  //if signal is received
  if (Radio.receiveDone()) {
    //send distances to transmitter
    Radio.sendWithRetry(4, distances, 3, 2, 20);
    //overwrite standard values with read values
    inputMotorSpeed = Radio.DATA[0];
    inputSteerAngle = Radio.DATA[1];
  }

  motorPWM(inputMotorSpeed);
  //if a click on the joystick is detected
  if (inputSteerAngle > 100) {
    WaterPistol.write(40); //spray watergun
  }
  else {
    steer(inputSteerAngle);
    WaterPistol.write(0); //reset watergun
  }
}

/**
 *  in Degrees°, 0 - 60
 *  steer from 60° to 120°
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
  analogWrite(MOTORPIN, dutyCycle * 2.55);
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
    // yield();
  }
  return false;
}
*/