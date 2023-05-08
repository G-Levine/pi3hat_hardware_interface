# pi3hat_hardware_interface

This repository provides a ros2_control hardware interface for the [mjbots pi3hat](https://github.com/mjbots/pi3hat).

## Prerequisites
- Raspberry Pi 4 running Ubuntu 22.04 and ROS2 Humble [(tested with this image)](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.1_v5.15.39-rt42-raspi_ros2_humble)
- Recommended: configure ros2_control for realtime operation as described [here](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html#determinism)
- Create a workspace `/home/pi/ros2_ws`

## Troubleshooting
### Issue
- Problem: `ImportError: libbcm_host.so.0: cannot open shared object file: No such file or directory`
- Solution: `sudo ln /usr/lib/aarch64-linux-gnu/libbcm_host.so /usr/lib/libbcm_host.so.0`

### Issue
- Problem: `pi3hat: could not open /dev/mem`
- Solution: `export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/ros2_ws/src/pi3hat_hardware_interface/fastrtps_profile_no_shmem.xml` and launch the ros2_control node as root with `sudo -E  ~/ros2_ws/src/pi3hat_hardware_interface/run_as_root.sh ros2 launch ~/ros2_ws/src/pi3hat_hardware_interface/test/test_state_publisher.launch.py`
- Explanation: The Pi3Hat library needs to be run as root to access the GPIO, but this prevents the FastDDS shared memory transport from working (no messages are published or received). Hence, we create a custom config file to force FastDDS to use UDP instead of shared memory.

### Issue
- Problem: Sometimes fails to start with `Segmentation fault (Address not mapped to object [(nil)])`
- Solution: TBD