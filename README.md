# SimpleFOC field stack example

Example firmware for driving a BLDC motor (not stepper) with 3 phases on the Funqi board module for the Field Stack driver.

It's also an example of how to use the STSPIN32G4 from PlatformIO.

Features:

- PlatformIO project
- Board configuration for working with generic STSPIN32G4 boards
- uses SimpleFOC and SimpleFOC drivers libraries
- Controls a BLDC motor
- Serial control via USB Serial and SimpleFOC's Commander class
- STSPIN32G4 driver for interacting with driver stage
- MT6835 driver for sensor
- STM32 CORDIC accellerated trig functions
- NeoPixel for status indication
- Stores motor calibration to internal Flash for fast start


## Software setup

Clone this repository, then switch to the lib folder and clone the dev branches of the SimpleFOC library and the SimpleFOC drivers library to that location.

Open the folder in PlatformIO.

Make sure your STM32 platform files are up-to-date.

