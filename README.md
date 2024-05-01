# StickerCutter Conversion Project

Large Format Printer Repurposing for Sticker Cutting

Project Overview

This project aims to repurpose a non-functional large format printer into a sticker cutting machine. The system will be controlled by a Raspberry Pi Pico rp2040 microcontroller board. An optical sensor on the former printhead will detect marks around printed stickers, and a blade on the printhead that originally cut the print job after completion will now cut strips of stickers. The project will be written in C and released as open-source on GitHub.

## Features

- **Mechanical Modifications:** The printer's existing printing head will be adapted to function as a sticker cutting head. This will involve adding an optical sensor to detect markers around printed stickers.
  
- **Software Control:** The Raspberry Pico rp2040 will control the cutting process. The software will interpret the signals from the optical sensor and instruct the cutting blade accordingly.
  
- **Language & Platform:** The project will be implemented in C language for the Raspberry Pico rp2040 microcontroller platform.
  
- **Open Source:** This project will be open source and hosted on GitHub, allowing for collaboration, contributions, and improvements from the community.

## PID Setting
* Speed controler: 
P = 5.0 
I = 4.0 
D = 3.0

* Position controler:
P = 50.0

## Velky motor - L404

* Red    +5V
* Black  GND
* BLue   A - OpenCollector
* white  B - OpenCollector


Maly motor - 
------------------

* Red    +5V
* Black  GND
* Green  A - OpenCollector
* white  B - OpenCollector
