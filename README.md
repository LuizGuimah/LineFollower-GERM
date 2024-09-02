# Line Follower - RISO/GERM/UDESC

This project is developed by a group of electrical engineering and computer science students.

## Hardware

The hardware is composed by an esp32-WROOM, a Pololu QTR-8RC sensor, two QTR-2 sensors, a TB6612FN motor driver and a pair of 10,000 rpm n20 motor with a 1:10 ratio gearbox.

## Software

The software is based on PID control logic, using discrete-time sampling to generate real-time corrections to keep the robot aligned and complete a white track on a black surface in the shortest possible time.

## Inputs

The software inputs consist of an array of 6 or 8 reading from the frontal QTR-8RC reflectance sensors, each reading has a range of 0-4095 (0 meaning a white surface and 4095 meaning no reflectance at all). Two lateral QTR-2 reflectance sensors are responsible for four digital inputs in which 1 stand for "On Line" and 0 "Not On Line"

## Outputs

The outputs of the current software are two PWM signal sent directly to the Motor Driver. Later will be added another PID function that will have the the first function as input representing the target speed at each motor and then aply a PID control to generate the PWM necessary to reach those speed faster.