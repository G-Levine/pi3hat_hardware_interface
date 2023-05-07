# pi3hat_hardware_interface

This repository provides a ros2_control hardware interface for the [mjbots pi3hat](https://github.com/mjbots/pi3hat).

## Prerequisites
- Raspberry Pi 4 running Ubuntu 22.04 and ROS2 Humble [(tested with this image)](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.1_v5.15.39-rt42-raspi_ros2_humble)

## Getting Ubuntu to work with the Pi3Hat
You will likely see this error:

`ImportError: libbcm_host.so.0: cannot open shared object file: No such file or directory`

Run this to solve it:

`sudo ln /usr/lib/aarch64-linux-gnu/libbcm_host.so /usr/lib/libbcm_host.so.0`