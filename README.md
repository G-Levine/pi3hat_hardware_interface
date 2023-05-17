# pi3hat_hardware_interface

This repository provides a ros2_control hardware interface for the [mjbots pi3hat](https://github.com/mjbots/pi3hat).

## Prerequisites
- Raspberry Pi 4 running Ubuntu 22.04 and ROS2 Humble [(tested with this image)](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.1_v5.15.39-rt42-raspi_ros2_humble)
- pi3hat r4.4 or newer (CAN doesn't seem to work on pi3hat r4.2)
- Recommended: configure ros2_control for realtime operation as described [here](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html#determinism)
- Create a workspace `/home/pi/ros2_ws`

## How to configure
- An example URDF is contained in `test_state_publisher.urdf.xacro`.
```
<ros2_control name="pi3hat_hardware_interface" type="system">
    <hardware>
        <plugin>pi3hat_hardware_interface/Pi3HatHardwareInterface</plugin>
        <param name="imu_mounting_deg.yaw">0</param>
        <param name="imu_mounting_deg.pitch">0</param>
        <param name="imu_mounting_deg.roll">0</param>
    </hardware>

    <joint name="joint_1">
        <param name="can_channel">1</param>
        <param name="can_id">1</param>
        <param name="can_protocol">cheetah</param>

        <param name="position_scale">95.5</param>
        <param name="velocity_scale">30.0</param>
        <param name="effort_scale">18.0</param>
        <param name="kp_scale">500.0</param>
        <param name="kd_scale">5.0</param>
        <param name="axis_direction">-1</param>
        <param name="position_offset">0.0</param>

        <command_interface name="position">
            <param name="min">-1</param>
            <param name="max">1</param>
        </command_interface>
        <command_interface name="velocity">
            <param name="min">-1</param>
            <param name="max">1</param>
        </command_interface>
        <command_interface name="effort">
            <param name="min">-1</param>
            <param name="max">1</param>
        </command_interface>
        <command_interface name="kp">
            <param name="min">-1</param>
            <param name="max">1</param>
        </command_interface>
        <command_interface name="kd">
            <param name="min">-1</param>
            <param name="max">1</param>
        </command_interface>

        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
    </joint>

    <sensor name="imu_sensor">
        <state_interface name="orientation.x"/>
        <state_interface name="orientation.y"/>
        <state_interface name="orientation.z"/>
        <state_interface name="orientation.w"/>
        <state_interface name="angular_velocity.x"/>
        <state_interface name="angular_velocity.y"/>
        <state_interface name="angular_velocity.z"/>
        <state_interface name="linear_acceleration.x"/>
        <state_interface name="linear_acceleration.y"/>
        <state_interface name="linear_acceleration.z"/>
    </sensor>
</ros2_control>
```
### Explanation of parameters
- `imu_mounting_deg.*`: Mounting angles for the IMU relative the the IMU link, measured in degrees
- `can_channel`: Which CANbus channel the actuator is connected to (must be between 1 and 5, inclusive)
- `can_id`: CANbus ID of the actuator (must be 1 or greater)
- `can_protocol`: Which protocol the actuator uses (only `cheetah` is implemented currently, works with driver boards running Mini Cheetah firmware such as SteadyWin and CubeMars)
- `*_scale`, `axis_direction`: Needed for encoding and decoding numeric values from CAN messages
- `position_offset`: Home position of the actuator in radians

## Troubleshooting
### Issue
- Problem: `ImportError: libbcm_host.so.0: cannot open shared object file: No such file or directory`
- Solution: `sudo ln /usr/lib/aarch64-linux-gnu/libbcm_host.so /usr/lib/libbcm_host.so.0`

### Issue
- Problem: `pi3hat: could not open /dev/mem`
- Solution: `export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/ros2_ws/src/pi3hat_hardware_interface/fastrtps_profile_no_shmem.xml` and launch the ros2_control node as root with `sudo -E bash ~/ros2_ws/src/pi3hat_hardware_interface/run_as_root.sh ros2 launch ~/ros2_ws/src/pi3hat_hardware_interface/test/test_state_publisher.launch.py`
- Explanation: The Pi3Hat library needs to be run as root to access the GPIO, but this prevents the FastDDS shared memory transport from working (no messages are published or received). Hence, we create a custom config file to force FastDDS to use UDP instead of shared memory.

### Issue
- Problem: Sometimes fails to start with `Segmentation fault (Address not mapped to object [(nil)])`
- Solution: Manually install ros2_control and associated packages from [this branch](https://github.com/schornakj/ros2_control/tree/pr-revert-922). Compile with `colcon build --symlink-install --allow-overriding controller_interface controller_manager hardware_interface ros2_control_test_assets`.
- Explanation: The controller manager suffers from a race condition on Humble [as described here](https://github.com/ros-controls/ros2_control/issues/979). The fix for this has not been merged yet.

## Debugging with GDB
```colcon build --packages-select pi3hat_hardware_interface --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo```

```sudo -E gdb --args /home/pi/ros2_ws/install/controller_manager/lib/controller_manager/ros2_control_node --ros-args --params-file /tmp/launch_params_sdkq8suy --params-file /home/pi/ros2_ws/src/pi3hat_hardware_interface/test/test_state_publisher.yaml```

```set environment LD_LIBRARY_PATH /home/pi/ros2_ws/install/transmission_interface/lib:/home/pi/ros2_ws/install/controller_manager/lib:/home/pi/ros2_ws/install/pi3hat_hardware_interface/lib:/home/pi/ros2_ws/install/controller_interface/lib:/home/pi/ros2_ws/install/hardware_interface/lib:/home/pi/ros2_ws/install/controller_manager_msgs/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib```