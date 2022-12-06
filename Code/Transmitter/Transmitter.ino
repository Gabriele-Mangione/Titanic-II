
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RFM69.h>
#include <SPI.h>

//Display settings
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3D
#define OLED_RESET -1

//Radio module pins
#define RFM69_RST 9
#define RFM69_CS 10
#define RFM69_INT 3

//Radio IDs
#define NETWORKID 0  // Must be the same for all nodes
#define MYNODEID 4   // My node ID

//prototypes
bool sendWithResponse(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime);

RFM69 radio(RFM69_CS, RFM69_INT);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    //for testing
    //Serial.begin(9600);

    //set pin modes
    pinMode(RFM69_RST, OUTPUT);

    digitalWrite(RFM69_RST, LOW);

    //display init
    while (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
    }
    delay(1000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);

    // RFM69 Radio init
    // manual Radio reset
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
    //read joystick values
    uint8_t outputMotorSpeed = map(analogRead(A0), 249, 772, 0, 200); //y axis
    uint8_t outputSteerAngle = map(analogRead(A1), 248, 775, 0, 90); //x axis

    //create buffer to send values via radio
    uint8_t buf[2] = { outputMotorSpeed, outputSteerAngle };
    uint8_t sensorDistances[3] = { 0 };

    //send and retry until a response has come
    if (sendWithResponse(3, buf, 2, 10, 20)) {
        radio.sendACK();
        //save distance values
        sensorDistances[0] = radio.DATA[0];
        sensorDistances[1] = radio.DATA[1];
        sensorDistances[2] = radio.DATA[2];
    }

    //display values
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
    display.setCursor(0, 32);
    display.print("Distance 0:  ");
    display.println(sensorDistances[0]);
    display.print("Distance 1:  ");
    display.println(sensorDistances[1]);
    display.print("Distance 2:  ");
    display.println(sensorDistances[2]);

    display.display();
}

/**
 * @brief send data and retry until getting a response back, in this case containing the sensor distances
 * 
 * @param toAddress adress of receiver radio
 * @param buffer data array to send
 * @param bufferSize size of data array
 * @param retries  how many times to retry sending
 * @param retryWaitTime how much time between retries in ms
 * @return true if a response has successfully been received
 * @return false when time is up and yet no responce has come
 */
bool sendWithResponse(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
    uint32_t sentTime;
    for (uint8_t i = 0; i <= retries; i++) {
        radio.send(toAddress, buffer, bufferSize, true);
        sentTime = millis();
        while (millis() - sentTime < retryWaitTime) {
            if (radio.receiveDone()) return true;
        }
    }
    return false;
}