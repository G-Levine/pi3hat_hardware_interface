#include "pi3hat_hardware_interface/pi3hat_hardware_interface.hpp"
#include "pi3hat_hardware_interface/pi3hat.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <sched.h>
#include <sys/mman.h>

#include <time.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEG_TO_RAD 0.01745329251994329576923690768489

namespace pi3hat_hardware_interface
{
    //----------------------------------------------------------------------
    // Hardware interface
    //----------------------------------------------------------------------

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Configure the Pi3Hat CAN for non-FD mode without bitrate switching or automatic retranmission
        mjbots::pi3hat::Pi3Hat::CanConfiguration can_config;
        can_config.fdcan_frame = false;
        can_config.bitrate_switch = false;
        can_config.automatic_retransmission = false;

        // Configure the Pi3Hat for 1000hz IMU sampling
        mjbots::pi3hat::Pi3Hat::Configuration config;
        config.attitude_rate_hz = 1000;

        // Set the mounting orientation of the IMU
        config.mounting_deg.yaw = 0;
        config.mounting_deg.pitch = 0;
        config.mounting_deg.roll = 0;

        // Set up the CAN configuration for all 5 CAN channels
        // for (size_t i = 0; i < 5; i++)
        // {
        //     config.can[i] = can_config;
        // }

        // Initialize the Pi3Hat
        pi3hat_ = new mjbots::pi3hat::Pi3Hat(config);

        // Initialize the Pi3Hat input
        pi3hat_input_ = mjbots::pi3hat::Pi3Hat::Input;

        hw_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        hw_command_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_kps_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_kds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Pi3HatHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Add joint state interfaces
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_efforts_[i]));
        }

        // Add IMU state interfaces
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "orientation.x", &hw_state_imu_orientation_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "orientation.y", &hw_state_imu_orientation_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "orientation.z", &hw_state_imu_orientation_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "orientation.w", &hw_state_imu_orientation_[3]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Pi3HatHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "kp", &hw_command_kps_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "kd", &hw_command_kds_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Pi3HatHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // Update the IMU state if attitude data is available
        if (pi3hat_input_.attitude)
        {
            hw_state_imu_orientation_[0] = pi3hat_input_.attitude->attitude.x;
            hw_state_imu_orientation_[1] = pi3hat_input_.attitude->attitude.y;
            hw_state_imu_orientation_[2] = pi3hat_input_.attitude->attitude.z;
            hw_state_imu_orientation_[3] = pi3hat_input_.attitude->attitude.w;
            hw_state_imu_angular_velocity_[0] = pi3hat_input_.attitude->rate_dps.x * DEG_TO_RAD;
            hw_state_imu_angular_velocity_[1] = pi3hat_input_.attitude->rate_dps.y * DEG_TO_RAD;
            hw_state_imu_angular_velocity_[2] = pi3hat_input_.attitude->rate_dps.z * DEG_TO_RAD;
            hw_state_imu_linear_acceleration_[0] = pi3hat_input_.attitude->accel_mps2.x;
            hw_state_imu_linear_acceleration_[1] = pi3hat_input_.attitude->accel_mps2.y;
            hw_state_imu_linear_acceleration_[2] = pi3hat_input_.attitude->accel_mps2.z;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type pi3hat_hardware_interface::Pi3HatHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        pi3hat_input_.request_attitude = true;
        pi3hat_input_.wait_for_attitude = true;

        const auto result = pi3hat_->Cycle(pi3hat_input_);

        if (result.error)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Pi3Hat::Cycle() failed!");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

} // namespace pi3hat_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    pi3hat_hardware_interface::Pi3HatHardwareInterface, hardware_interface::SystemInterface)