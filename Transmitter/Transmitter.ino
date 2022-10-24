
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RFM69.h>
#include <SPI.h>
#include <Wire.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3D
#define OLED_RESET -1

#define RFM69_RST 9
#define RFM69_CS 10
#define RFM69_INT 3
#define NETWORKID 0  // Must be the same for all nodes
#define MYNODEID 4   // My node ID
// Arduino pins

RFM69 radio(RFM69_CS, RFM69_INT);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// HC12 Radio-module pins
//#define HC12TX 6
//#define HC12RX 5

// SoftwareSerial HC12(HC12TX, HC12RX); // Define HC12 communication pins

void setup() {
  Serial.begin(9600);
  // HC12-Radiomodule
  // HC12.begin(9600);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);

  Serial.println("RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!radio.initialize(RF69_915MHZ, MYNODEID, NETWORKID)) {
    Serial.println("RFM69 radio init failed");
    while (1)
      ;
  }
  radio.setHighPower();
}

void loop() {
  uint8_t outputMotorSpeed = map(analogRead(A0), 249, 772, 0, 200);
  uint8_t outputSteerAngle = map(analogRead(A1), 248, 775, 0, 90);

  uint8_t buf[2];
  uint8_t len = sizeof(buf);

  display.clearDisplay();
  display.setCursor(0, 5);
  display.print("Motor Speed:     ");
  display.print(outputMotorSpeed - 100);
  display.setCursor(120, 5);
  display.println("%");
  display.print("Steering Angle:  ");
  display.print(outputSteerAngle + 45);
  display.setCursor(120, 11);
  display.println("o");
  /*
    if (timeoutReceiver(100)) {
      if (radio.DATA[0] = 1) {
        Serial.println("success");
        Serial.println(analogRead(A0));
        Serial.println(analogRead(A1));
        buf[0] = outputMotorSpeed;
        buf[1] = outputSteerAngle;
        radio.send(3, buf, 2);
      }
    }*/
  Serial.println(analogRead(A0));
  Serial.println(analogRead(A1));
  buf[0] = outputMotorSpeed;
  buf[1] = outputSteerAngle;
  radio.send(3, buf, 2);
  /*if (HC12.available()) {
      if (HC12.read() == 1) {
          HC12.print(outputMotorSpeed || outputSteerAngle << 7);
      }
  }*/
  display.display();
}
bool timeoutReceiver(unsigned long stoptime) {
  unsigned long time = millis();
  while (millis() - time < stoptime) {
    if (radio.receiveDone()) {
      return true;
    }
    yield();
  }
  return false;
}