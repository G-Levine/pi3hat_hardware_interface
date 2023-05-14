#ifndef PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_HPP_
#define PI3HAT_HARDWARE_INTERFACE__PI3HAT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pi3hat/pi3hat.h"

#define MAX_NUM_CAN_FRAMES 12

namespace pi3hat_hardware_interface
{
    class Pi3HatHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Pi3HatHardwareInterface)

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        int float_to_uint(float x, float x_min, float x_max,
                          int bits);
        float uint_to_float(int x_int, float x_min, float x_max,
                            int bits);

        // Enum for CAN protocol types: cheetah, myactuator, or moteus
        enum class CanProtocol
        {
            CHEETAH,    // MIT Mini Cheetah actuators, SteadyWin, CubeMars
            MYACTUATOR, // MyActuator, LKMTech
            MOTEUS      // Moteus (non-FD CAN)
        };

        const unsigned char cheetahEnableMsg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                   0xFF, 0xFF, 0xFF, 0xFC};

        const unsigned char cheetahDisableMsg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                    0xFF, 0xFF, 0xFF, 0xFD};

        const unsigned char cheetahSetZeroPositionMsg[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                                            0xFF, 0xFF, 0xFF, 0xFE};

        const unsigned char cheetahSetIdleCmdMsg[8] = {0x7F, 0xFF, 0x7F, 0xF0,
                                                       0x00, 0x00, 0x07, 0xFF};

        mjbots::pi3hat::Pi3Hat *pi3hat_;
        mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;
        mjbots::pi3hat::Attitude attitude_;
        mjbots::pi3hat::CanFrame tx_can_frames_[MAX_NUM_CAN_FRAMES];
        mjbots::pi3hat::CanFrame rx_can_frames_[MAX_NUM_CAN_FRAMES];

        // IMU state
        std::array<double, 4> hw_state_imu_orientation_;         // x, y, z, w
        std::array<double, 3> hw_state_imu_angular_velocity_;    // x, y, z
        std::array<double, 3> hw_state_imu_linear_acceleration_; // x, y, z

        // Actuator CAN config
        std::vector<int> hw_actuator_can_channels_;
        std::vector<int> hw_actuator_can_ids_;
        std::vector<CanProtocol> hw_actuator_can_protocols_;

        // Actuator parameters
        std::vector<double> hw_actuator_position_scales_;
        std::vector<double> hw_actuator_velocity_scales_;
        std::vector<double> hw_actuator_effort_scales_;
        std::vector<double> hw_actuator_kp_scales_;
        std::vector<double> hw_actuator_kd_scales_;
        std::vector<int> hw_actuator_axis_directions_;
        std::vector<double> hw_actuator_position_offsets_;

        // Actuator states
        std::vector<double> hw_state_positions_;
        std::vector<double> hw_state_velocities_;
        std::vector<double> hw_state_efforts_;

        // Actuator commands
        std::vector<double> hw_command_positions_;
        std::vector<double> hw_command_velocities_;
        std::vector<double> hw_command_efforts_;
        std::vector<double> hw_command_kps_;
        std::vector<double> hw_command_kds_;
    };

} // namespace pi3hat_hardware_interface

#endif // PI3HAT_HARDWARE_INTERFACE_PUBLIC__PI3HAT_HARDWARE_INTERFACE_HPP_