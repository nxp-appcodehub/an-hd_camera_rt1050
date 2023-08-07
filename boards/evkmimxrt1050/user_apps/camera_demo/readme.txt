Overview
========
The camera demo shows how to receive the camera data using CSI driver.
In this example, you will see the camera frames shown in the LCD. Optionally,
you can touch the LCD panel with you finger, the current frame will be compressed
to JPEG format and stored in the microSD card.

Toolchain supported
===================
- IAR embedded Workbench 8.20

Hardware requirements
=====================
- EVK-MIMXRT1050 board
- MT9M114 camera module
- LCD8000-43T LCD module
- microSD card
- Personal Computer
- Micro USB cable

Board settings
==============
1. Connect the LCD8000-43T module.
2. Connect the MT9M114 camera module.
3. Insert the microSD card.

Prepare the Demo
================
1.  Connect a USB cable between the host PC and the OpenSDA USB port on the EVK board.
2.  Optionally, open a serial terminal with the following settings to see the logs.
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Launch the debugger in your IDE to begin running the demo.

Running the demo
================
When the demo runs successfully, the camera received pictures are shown in the LCD.
You can touch the LCD panel with your finger, the current frame will be compressed
to JPEG format and stored in the microSD card.

