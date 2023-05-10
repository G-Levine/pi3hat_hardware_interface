#include "pi3hat_hardware_interface/pi3hat_hardware_interface.hpp"

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
    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

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
            std::string can_protocol_string = joint.parameters["can_protocol"];
            switch (can_protocol_string)
            {
            case "cheetah":
                hw_actuator_can_protocols_.push_back(CanProtocol::CHEETAH);
                break;
            case "myactuator":
                hw_actuator_can_protocols_.push_back(CanProtocol::MYACTUATOR);
                break;
            case "moteus":
                hw_actuator_can_protocols_.push_back(CanProtocol::MOTEUS);
                break;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "can_protocol parameter %s does not match a valid protocol", can_protocol_string);
                return hardware_interface::return_type::ERROR;
            }

            hw_actuator_can_channels_.push_back(joint.parameters["can_channel"]);
            hw_actuator_can_ids_.push_back(joint.parameters["can_id"]);
            hw_actuator_position_scales_.push_back(joint.parameters["position_scale"]);
            hw_actuator_velocity_scales_.push_back(joint.parameters["velocity_scale"]);
            hw_actuator_effort_scales_.push_back(joint.parameters["effort_scale"]);
            hw_actuator_kp_scales_.push_back(joint.parameters["kp_scale"]);
            hw_actuator_kd_scales_.push_back(joint.parameters["kd_scale"]);
            hw_actuator_axis_directions_.push_back(joint.parameters["axis_direction"]);
            hw_actuator_position_offsets_.push_back(joint.parameters["position_offset"]);
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

        // Initialize the Pi3Hat input
        pi3hat_input_ = mjbots::pi3hat::Pi3Hat::Input();
        mjbots::pi3hat::Attitude attitude_;
        pi3hat_input_.attitude = &attitude_;

        // Set up the CAN configuration
        for (uint i = 0; i < hw_actuator_can_channels_.size(); i++)
        {
            config.can[hw_actuator_can_channels_[i]] = can_config;
            pi3hat_input_.tx_can[i].id = hw_actuator_can_ids_[i];
            pi3hat_input_.tx_can[i].bus = hw_actuator_can_channels_[i];
            pi3hat_input_.tx_can[i].expect_reply = true;
        }

        // Initialize the Pi3Hat
        pi3hat_ = new mjbots::pi3hat::Pi3Hat(config);

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
            "imu_sensor", "orientation.x", &hw_state_imu_orientation_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "orientation.y", &hw_state_imu_orientation_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "orientation.z", &hw_state_imu_orientation_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "orientation.w", &hw_state_imu_orientation_[3]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "imu_sensor", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));

        return state_interfaces;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // reset values always when configuring hardware
        for (uint i = 0; i < hw_state_positions_.size(); i++)
        {
            hw_state_positions_[i] = 0;
            hw_state_velocities_[i] = 0;
            hw_state_efforts_[i] = 0;
            hw_command_positions_[i] = 0;
            hw_command_velocities_[i] = 0;
            hw_command_efforts_[i] = 0;
            hw_command_kps_[i] = 0;
            hw_command_kds_[i] = 0;
        }

        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            switch (hw_actuator_can_protocols_)
            {
            case CanProtocol::CHEETAH:
                std::copy(std::begin(cheetahSetZeroPositionMsg), std::end(cheetahSetZeroPositionMsg), std::begin(pi3hat_input_.rx_can[i].data));
                break;
            }
        }
        const auto result = pi3hat_->Cycle(pi3hat_input_);

        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
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
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            switch (hw_actuator_can_protocols_)
            {
            case CanProtocol::CHEETAH:
                pi3hat_input_.rx_can[i].data = cheetahEnableMsg;
                std::copy(std::begin(cheetahEnableMsg), std::end(cheetahEnableMsg), std::begin(pi3hat_input_.rx_can[i].data));
                break;
            }
        }
        const auto result = pi3hat_->Cycle(pi3hat_input_);

        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            switch (hw_actuator_can_protocols_)
            {
            case CanProtocol::CHEETAH:
                // We send a command of all zeroes to the actuator efore disabling it
                // This is to prevent the actuator from moving if we re-enable it later
                std::copy(std::begin(cheetahSetIdleCmdMsg), std::end(cheetahSetIdleCmdMsg), std::begin(pi3hat_input_.rx_can[i].data));
                break;
            }
        }
        const auto result = pi3hat_->Cycle(pi3hat_input_);

        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            switch (hw_actuator_can_protocols_)
            {
            case CanProtocol::CHEETAH:
                std::copy(std::begin(cheetahDisableMsg), std::end(cheetahDisableMsg), std::begin(pi3hat_input_.rx_can[i].data));
                break;
            }
        }
        const auto result = pi3hat_->Cycle(pi3hat_input_);

        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Pi3HatHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // Reading is done in the write() method
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Pi3HatHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            switch (hw_actuator_can_protocols_)
            {
            case CanProtocol::CHEETAH:
                float p_des = hw_command_positions_[i] * hw_actuator_axis_directions_[i] - hw_actuator_position_offsets_[i];
                float v_des = hw_command_velocities_[i] * hw_actuator_axis_directions_[i];
                float tau_ff = hw_command_efforts_[i] * hw_actuator_axis_directions_[i];

                // Apply Saturation based on the limits
                p_des = fminf(fmaxf(-hw_actuator_position_scales_[i], p_des), hw_actuator_position_scales_[i]);
                v_des = fminf(fmaxf(-hw_actuator_velocity_scales_[i], v_des), hw_actuator_velocity_scales_[i]);
                tau_ff = fminf(fmaxf(-hw_actuator_effort_scales_[i], tau_ff), hw_actuator_effort_scales_[i]);
                float kp = fminf(fmaxf(0.0, hw_command_kps_[i]), hw_actuator_kp_scales_[i]);
                float kd = fminf(fmaxf(0.0, hw_command_kds_[i]), hw_actuator_kd_scales_[i]);

                // convert floats to unsigned ints
                int p_int =
                    float_to_uint(p_des, -hw_actuator_position_scales_[i], hw_actuator_position_scales_[i], 16);
                int v_int =
                    float_to_uint(v_des, -hw_actuator_velocity_scales_[i], hw_actuator_velocity_scales_[i], 12);
                int kp_int =
                    float_to_uint(kp, 0.0, hw_actuator_kp_scales_[i], 12);
                int kd_int =
                    float_to_uint(kd, 0.0, hw_actuator_kd_scales_[i], 12);
                int t_int =
                    float_to_uint(tau_ff, -hw_actuator_effort_scales_[i], hw_actuator_effort_scales_[i], 12);

                // pack ints into the can message
                pi3hat_input_.tx_can.data[0] = p_int >> 8;
                pi3hat_input_.tx_can.data[1] = p_int & 0xFF;
                pi3hat_input_.tx_can.data[2] = v_int >> 4;
                pi3hat_input_.tx_can.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
                pi3hat_input_.tx_can.data[4] = kp_int & 0xFF;
                pi3hat_input_.tx_can.data[5] = kd_int >> 4;
                pi3hat_input_.tx_can.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
                pi3hat_input_.tx_can.data[7] = t_int & 0xff;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Unknown CAN protocol: %d", hw_actuator_can_protocols_);
                break;
            }
        }

        pi3hat_input_.request_attitude = true;
        pi3hat_input_.wait_for_attitude = true;

        const auto result = pi3hat_->Cycle(pi3hat_input_);

        if (result.error)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Pi3Hat::Cycle() failed!");
            return hardware_interface::return_type::ERROR;
        }

        // Update the IMU state if attitude data is available
        if (result.attitude_present)
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

        // Read and update the actuator states if data is available
        if (result.rx_can_size > 0)
        {
            for (auto i = 0u; i < hw_state_positions_.size(); i++)
            {
                for (auto j = 0u; j < result.rx_can_size; j++)
                {
                    switch (hw_actuator_can_protocols_)
                    {
                    case CanProtocol::CHEETAH:
                        int id = pi3hat_input_.rx_frames.at(j)[0];
                        if (id == hw_actuator_can_ids_[i])
                        {
                            // parse the can frame
                            int p_int = (pi3hat_input_.rx_frames.at(j)[1] << 8) | pi3hat_input_.rx_frames.at(j)[2];
                            int v_int = (pi3hat_input_.rx_frames.at(j)[3] << 4) | (pi3hat_input_.rx_frames.at(j)[4] >> 4);
                            int i_int = ((pi3hat_input_.rx_frames.at(j)[4] & 0xF) << 8) | pi3hat_input_.rx_frames.at(j)[5];

                            // convert unsigned ints to floats
                            hw_state_positions_[i] = uint_to_float(p_int, -hw_actuator_position_scales_[i], hw_actuator_position_scales_[i], 16) * hw_actuator_position_directions_[i] + hw_actuator_position_offsets_[i];
                            hw_state_velocities_[i] = uint_to_float(v_int, -hw_actuator_velocity_scales_[i], hw_actuator_velocity_scales_[i], 12) * hw_actuator_position_directions_[i];
                            hw_state_efforts_[i] = uint_to_float(i_int, -hw_actuator_torque_scales_[i], hw_actuator_torque_scales_[i], 12) * hw_actuator_position_directions_[i];
                        }
                        break;
                    }
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    int float_to_uint(float x, float x_min, float x_max,
                      int bits) const
    {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    float uint_to_float(int x_int, float x_min, float x_max,
                        int bits) const
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

} // namespace pi3hat_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    pi3hat_hardware_interface::Pi3HatHardwareInterface, hardware_interface::SystemInterface)