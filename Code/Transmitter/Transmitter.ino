
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>
#include <SPI.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3D
#define OLED_RESET -1

//Radio module pins
#define RFM69_RST 9
#define RFM69_CS 10
#define RFM69_INT 3

//Radio IDs
#define MYNODEID 4   // My node ID

RFM69 Radio(RFM69_CS, RFM69_INT);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t sensorDistances[NUMBER_OF_SENSORS] = { 0 };
uint32_t lastRadioCommunication = 0;

void setup() {
    Serial.begin(9600); //for testing

    // RFM69 Radio init
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, HIGH);  // manual reset
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    while (!Radio.initialize(RF69_915MHZ, MYNODEID, NETWORKID)) {
        Serial.println("RFM69 radio init failed");
    }
    Radio.setHighPower();
    Radio.setIsrCallback(myCallback);  // set myCallback() function to call in the interrupt function of the radio library
    Radio.setMode(RF69_MODE_RX);

    // display setup
    while (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
    }
    delay(1000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
}

void loop() {
    Serial.println("Heyo allocation failed");
    // read Joystick inputs
    uint8_t outputMotorSpeed = map(analogRead(A0), 249, 772, 0, 200);
    uint8_t outputSteerAngle = map(analogRead(A1), 248, 775, 0, 90);
    uint8_t buf[2] = { outputMotorSpeed, outputSteerAngle };
    Serial.println("1");
    Radio.send(3,buf,2);
    /*
    Radio.writeReg(REG_PACKETCONFIG2, (Radio.readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART);  // avoid RX deadlocks
    // send values of speed and angle to the boat
    if (Radio.canSend()) {
        Serial.println("5");


        Radio.sendFrame(3, buf, 2);
        Serial.println("6");
        Radio.setMode(RF69_MODE_RX);
        Serial.println("2");
    }
    */

    // print sent values
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
    Serial.println("3");

    // display distances of ultrasound sensors
    display.setCursor(0, 32);
    for (uint8_t i = 1; i < NUMBER_OF_SENSORS; i++) {
        display.print("Distance ");
        display.print(i);
        display.print(":  ");
        display.println(sensorDistances[i - 1]);
    }

    //display time since last message from boat
    display.setCursor(0, 58);
    display.print(millis() - lastRadioCommunication);

    display.display();
}

void myCallback() {
    Serial.println("weyo");
    Radio.setMode(RF69_MODE_STANDBY);
    for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
        sensorDistances[i] = Radio.DATA[i];
    }
    lastRadioCommunication = millis();
    Radio.setMode(RF69_MODE_RX);
}
