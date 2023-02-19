# pi3hat_hardware_interface

This repository provides a ros2_control hardware interface for the [mjbots pi3hat](https://github.com/mjbots/pi3hat).

## Features
- Configure the CAN channel and arbitration ID for each actuator
- Encode and decode MIT Cheetah motor driver messages (position, velocity, feedforward torque, Kp, Kd)
- Send and receive CAN messages for up to 3 actuators per channel / 5 channels
- Read IMU state
