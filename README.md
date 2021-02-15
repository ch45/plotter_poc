# plotter_poc - Proof of Concept for ESP32 Plotter controller

This is just a proof of concept - only connect the components away from a plotting machine!

## Components

* ESP32 DevKitC ESP32-WROOM
* DRV8825 Stepper Motor Driver Carriers x 2
* Stepper Motors NEMA 17 x 2
* 12V 1A PSU
* Solenoid and driver module (optional, not yet implemented)

## Method

* Uses the ESP32 FreeRTOS Hardware timers to toggle the GPIO states for the Step signal to the stepper drivers at different rates.

## Specification

* Maximum 1000Hz (limited by the stepper motors)

## Work in Progress

* Complete the stepper_run control
* Read coordinate data and use this to control the hardware timer counting
* Read ahead data to produce synchronised accelartion and deceleration ramp data for the PWM
* Implement the pen solenoid / servo signal
* Implement the limit stops and re-position to origin.
