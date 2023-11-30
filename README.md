# Picasso_MyActuator_Control

This repo contains microcontroller control code for controlling MyActuator QDD actuators. This microcontroller code is written for Arduino compatible boards, specifically the Teensy 4.1. We will be working on supporting more microcontrollers in future.

The repo also has a python header program that can be used to do some basic torque tracking to get you started. This code requires a serial (UART) connection between your microcontroller and the pc running the python program.

The src file contains the driver files (written in c++ with a header file for conversion to arduino) for basic motor control. We also have some examples of slightly friendlier microcontroller code written for particular models of motor.
