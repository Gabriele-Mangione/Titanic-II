# TITANIC II
TITANIC II is a project for an RC boat model with paired controller made by me!


## Working principle

Both electronics are powered using 14.4V Batteries, use Arduino Nano Every(s) as μController and communicate via LoRa using two RFM69HCW modules.
The Boat uses a 12V motor to swim and a servo-motor to steer, while the transmitter sends information gathered by a joystick module. 

## Features 
The boat is equipped with:
* A water gun controlled by a servomotor when clicking on the joystick
* Adjustable PWM motor speed
* Three HC-SR04 ultrasonic sensors that prevent hard collisions through adjusting of the motor speed
* External power supply pins for implementation of further modules (i.e. solar panels)

The transmitter is equipped with:
* Joystick module for a good user interface
* A tft 128x64 monochrome display to show inputs sent to the boat and boat connection
* Comfortable 3d-printed case

## Dependencies
The codes need the following libraries in order to work.

Boat.ino:
* RFM69 lib (Lowpowerlab)
* SPI lib (Arduino)
* Servo lib (Arduino)
* Ultrasonicsensor lib (made by me)
  * you can find this in my GitHub Repositories

Transmitter.ino:
* GFX lib (Adafruit)
* SSD1306 lib (Adafruit)
* RFM69 lib (Lowpowerlab)
* SPI lib (Arduino)

## Contributing

You are welcome to make commits of suggestions for improvements.

## License

[MIT](https://choosealicense.com/licenses/mit/)
