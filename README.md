# RainingPhotons firmware
Firmware for light festival project

The main processor is a STM32 (BluePill), running in Arduino mode.
[STM32 Arduino](https://github.com/rogerclarkmelbourne/Arduino_STM32)

Hooked up via SPI is an ethernet board (ENC28J60) that is programmed with the UPIEthernet library.
[Github Repo](https://github.com/UIPEthernet/UIPEthernet.git)

Off of the i2c bus is an Adafruit LIS3DH Accelerometer breakout board.
[Github Repo](https://github.com/adafruit/Adafruit_LIS3DH.git)
[Adafruit info](https://www.adafruit.com/product/2809)

Connected to GPIO pins are strips of SK6812 LEDS programmed using the FastLED library.
[Github Repo](https://github.com/evq/FastLED.git)