
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>
#include <SPI.h>

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

RFM69 Radio(RFM69_CS, RFM69_INT);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t sensorDistances[3] = {0};

void setup() {
    Serial.begin(9600);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    // RFM69 Radio init
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    while (!Radio.initialize(RF69_915MHZ, MYNODEID, NETWORKID)) {
        Serial.println("RFM69 radio init failed");
    }
    Radio.setHighPower();
    Radio.setIsrCallback(myCallback);

    while (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
    }
    delay(1000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
}

void loop() {
    uint8_t outputMotorSpeed = map(analogRead(A0), 249, 772, 0, 200);
    uint8_t outputSteerAngle = map(analogRead(A1), 248, 775, 0, 90);
    uint8_t buf[2] = {outputMotorSpeed, outputSteerAngle};

    Radio.send(3, buf, 2);

    display.clearDisplay();
    display.setCursor(0, 5);
    display.print("Motor Speed:     ");
    display.print(outputMotorSpeed - 100);
    display.setCursor(120, 5);
    display.println("%");
    display.print("Steering Angle:  ");
    display.print(outputSteerAngle - 45);
    display.setCursor(120, 11);
    display.println("o");

    display.setCursor(0, 32);
    display.print("Distance 0:  ");
    display.println(sensorDistances[0]);
    display.print("Distance 1:  ");
    display.println(sensorDistances[1]);
    display.print("Distance 2:  ");
    display.println(sensorDistances[2]);

    display.display();
}

void myCallback() {
    sensorDistances[0] = Radio.DATA[0];
    sensorDistances[1] = Radio.DATA[1];
    sensorDistances[2] = Radio.DATA[2];
}
