# Car Setup

## High Level Overview

The platform is based off of the Traxxas 1/10 scale Slash RC truck, with as many components being kept stock / unmodified as possible. 

<br>

The car's computer is an NVIDIA Jetson Orin Nano Super, with an adafruit LIDAR, IMU, and camera acting as peripherals. The computer is accessed through SSH, where ROS2 Humble running on a docker container with Ubuntu 22.04 is used manage everything. 

<br>

The Jetson is connected to the traxxas tqi radio receiver, where it reads PWM pulses and interprets them as commands for the RC car, acting as a manual override. The Jetson is also connected to a PCA9685 board, where the Electronic Speed Controller and steering servo are connected and sent commands accordingly. The Jetson acts as a multiplexer, and decides to drive the car via the radio controller or via onboard autonomy. During an autonomous run, any control input would be given priority, and cause the autonomous control to halt.