# TITANIC II :ship:
TITANIC II is a project for an RC boat model with paired controller made by me! :grin:


## Working principle :gear:

Both electronics are powered using 14.4V Batteries, use Arduino Nano Every(s) as ╬╝Controller and communicate via LoRa using two RFM69HCW modules.
The Boat uses a 12V motor to swim and a servo-motor to steer, while the transmitter sends information gathered by a joystick module. 

## Features :bulb:
The boat is equipped with:
* A water gun controlled by a servomotor when clicking on the joystick
* Adjustable PWM motor speed
* Three HC-SR04 ultrasonic sensors that prevent hard collisions through adjusting of the motor speed
* External power supply pins for implementation of further modules (i.e. solar panels)

The transmitter is equipped with:
* Joystick module for a good user interface
* A tft 128x64 monochrome display to show inputs sent to the boat and boat connection
* Comfortable 3d-printed case

## Dependencies :scroll:
The codes need the following libraries in order to work.

Boat.ino:
* RFM69 lib (Lowpowerlab)
* SPI lib (Arduino)
* Servo lib (Arduino)
* [Ultrasonicsensor lib (made by me)](https://github.com/Gabriele-Mangione/UltrasonicSensor_lib)
  * you can find this in [my GitHub Repositories](https://github.com/Gabriele-Mangione?tab=repositories)

Transmitter.ino:
* GFX lib (Adafruit)
* SSD1306 lib (Adafruit)
* RFM69 lib (Lowpowerlab)
* SPI lib (Arduino)

## Contributing :pencil2:

You are welcome to make commits of suggestions for improvements.

## License :bookmark_tabs:

[MIT](https://choosealicense.com/licenses/mit/)
