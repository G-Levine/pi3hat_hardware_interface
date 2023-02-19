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

#include "pi3hat.h"

namespace pi3hat_hardware_interface
{
    class Pi3HatHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Pi3HatHardware)

        struct ImpedanceCommand {
            double position;
            double velocity;
            double effort;
            double Kp;
            double Kd;
        };

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

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
        mjbots::pi3hat::Pi3Hat* pi3hat_;
        mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;

        // IMU state
        std::array<double, 4> hw_state_imu_orientation_; // x, y, z, w
        std::array<double, 3> hw_state_imu_angular_velocity_; // x, y, z
        std::array<double, 3> hw_state_imu_linear_acceleration_; // x, y, z

        // Actuator parameters
        std::vector<int> hw_actuator_can_channels_;
        std::vector<int> hw_actuator_can_ids_;
        std::vector<int> hw_actuator_directions_;

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

} // namespace ros2_control_demo_hardware

#endif // PI3HAT_HARDWARE_INTERFACE_PUBLIC__PI3HAT_HARDWARE_INTERFACE_HPP_