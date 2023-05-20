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
            if ("cheetah" == joint.parameters.at("can_protocol"))
            {
                hw_actuator_can_protocols_.push_back(CanProtocol::CHEETAH);
            }
            else if ("myactuator" == joint.parameters.at("can_protocol"))
            {
                hw_actuator_can_protocols_.push_back(CanProtocol::MYACTUATOR);
            }
            else if ("moteus" == joint.parameters.at("can_protocol"))
            {
                hw_actuator_can_protocols_.push_back(CanProtocol::MOTEUS);
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "can_protocol parameter does not match a valid protocol");
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Set params for each joint
            hw_actuator_can_channels_.push_back(std::stoi(joint.parameters.at("can_channel")));
            hw_actuator_can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
            hw_actuator_position_scales_.push_back(std::stod(joint.parameters.at("position_scale")));
            hw_actuator_velocity_scales_.push_back(std::stod(joint.parameters.at("velocity_scale")));
            hw_actuator_effort_scales_.push_back(std::stod(joint.parameters.at("effort_scale")));
            hw_actuator_kp_scales_.push_back(std::stod(joint.parameters.at("kp_scale")));
            hw_actuator_kd_scales_.push_back(std::stod(joint.parameters.at("kd_scale")));
            hw_actuator_axis_directions_.push_back(std::stoi(joint.parameters.at("axis_direction")));
            hw_actuator_position_offsets_.push_back(std::stod(joint.parameters.at("position_offset")));

            // Set limits for each joint
            hw_actuator_position_mins_.push_back(std::stod(joint.parameters.at("position_min")));
            hw_actuator_position_maxs_.push_back(std::stod(joint.parameters.at("position_max")));
            hw_actuator_velocity_maxs_.push_back(std::stod(joint.parameters.at("velocity_max")));
            hw_actuator_effort_maxs_.push_back(std::stod(joint.parameters.at("effort_max")));
            hw_actuator_kp_maxs_.push_back(std::stod(joint.parameters.at("kp_max")));
            hw_actuator_kd_maxs_.push_back(std::stod(joint.parameters.at("kd_max")));
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
        config.mounting_deg.yaw = std::stod(info_.hardware_parameters.at("imu_mounting_deg.yaw"));
        config.mounting_deg.pitch = std::stod(info_.hardware_parameters.at("imu_mounting_deg.pitch"));
        config.mounting_deg.roll = std::stod(info_.hardware_parameters.at("imu_mounting_deg.roll"));

        // Initialize the Pi3Hat input
        pi3hat_input_ = mjbots::pi3hat::Pi3Hat::Input();
        pi3hat_input_.attitude = &attitude_;
        mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_can_frames_span_(rx_can_frames_, MAX_NUM_CAN_FRAMES); 
        pi3hat_input_.rx_can = rx_can_frames_span_;
        mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_can_frames_span_(tx_can_frames_, info_.joints.size()); 
        pi3hat_input_.tx_can = tx_can_frames_span_;

        // Set up the CAN configuration
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            config.can[hw_actuator_can_channels_[i] - 1] = can_config;
            pi3hat_input_.tx_can[i].id = hw_actuator_can_ids_[i];
            pi3hat_input_.tx_can[i].bus = hw_actuator_can_channels_[i];
            pi3hat_input_.tx_can[i].expect_reply = true;
            pi3hat_input_.tx_can[i].size = 8;
        }

        // Initialize the Pi3Hat
        pi3hat_ = new mjbots::pi3hat::Pi3Hat(config);

        // Configure realtime scheduling
        {
            int realtime_cpu = 0;
            cpu_set_t cpuset = {};
            CPU_ZERO(&cpuset);
            CPU_SET(realtime_cpu, &cpuset);

            const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
            if (r < 0) {
            throw std::runtime_error("Error setting CPU affinity");
            }

            std::cout << "Affinity set to " << realtime_cpu << "\n";
        }
        {
            struct sched_param params = {};
            params.sched_priority = 10;
            const int r = ::sched_setscheduler(0, SCHED_RR, &params);
            if (r < 0) {
            throw std::runtime_error("Error setting realtime scheduler");
            }
        }
        {
            const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
            if (r < 0) {
            throw std::runtime_error("Error locking memory");
            }
        }

        sleep(2);
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            switch (hw_actuator_can_protocols_[i])
            {
            case CanProtocol::CHEETAH:
                std::copy(std::begin(cheetahSetZeroPositionMsg), std::end(cheetahSetZeroPositionMsg), std::begin(pi3hat_input_.tx_can[i].data));
                break;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to start: unknown CAN protocol");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        pi3hat_->Cycle(pi3hat_input_);
        sleep(2);

        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            switch (hw_actuator_can_protocols_[i])
            {
            case CanProtocol::CHEETAH:
                std::copy(std::begin(cheetahEnableMsg), std::end(cheetahEnableMsg), std::begin(pi3hat_input_.tx_can[i].data));
                break;
            default:
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to start: unknown CAN protocol");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        pi3hat_->Cycle(pi3hat_input_);
        sleep(2);

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
        RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Pi3HatHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            switch (hw_actuator_can_protocols_[i])
            {
            case CanProtocol::CHEETAH:
                // We send a command of all zeroes to the actuator before disabling it
                // This is to prevent the actuator from moving if we re-enable it later
                std::copy(std::begin(cheetahSetIdleCmdMsg), std::end(cheetahSetIdleCmdMsg), std::begin(pi3hat_input_.tx_can[i].data));
                break;
            }
        }
        pi3hat_->Cycle(pi3hat_input_);
        sleep(2);
        for (auto i = 0u; i < hw_state_positions_.size(); i++)
        {
            switch (hw_actuator_can_protocols_[i])
            {
            case CanProtocol::CHEETAH:
                std::copy(std::begin(cheetahDisableMsg), std::end(cheetahDisableMsg), std::begin(pi3hat_input_.tx_can[i].data));
                break;
            }
        }
        pi3hat_->Cycle(pi3hat_input_);
        sleep(2);

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
            if (std::isnan(hw_command_positions_[i]) || std::isnan(hw_command_velocities_[i]) || std::isnan(hw_command_efforts_[i]) || std::isnan(hw_command_kps_[i]) || std::isnan(hw_command_kds_[i]))
            {
                RCLCPP_WARN(rclcpp::get_logger("Pi3HatHardwareInterface"), "NaN command for actuator");
                continue;
            }

            switch (hw_actuator_can_protocols_[i])
            {
            case CanProtocol::CHEETAH:
                {
                    // constrain commands to the user-defined limits
                    double p_des = fminf(fmaxf(hw_actuator_position_mins_[i], hw_command_positions_[i]), hw_actuator_position_maxs_[i]);
                    double v_des = fminf(fmaxf(-hw_actuator_velocity_maxs_[i], hw_command_velocities_[i]), hw_actuator_velocity_maxs_[i]);
                    double tau_ff = fminf(fmaxf(-hw_actuator_effort_maxs_[i], hw_command_efforts_[i]), hw_actuator_effort_maxs_[i]);
                    double kp = fminf(fmaxf(0.0, hw_command_kps_[i]), hw_actuator_kp_maxs_[i]);
                    double kd = fminf(fmaxf(0.0, hw_command_kds_[i]), hw_actuator_kd_maxs_[i]);

                    // compensate for axis directions and offsets
                    p_des = (p_des - hw_actuator_position_offsets_[i]) * hw_actuator_axis_directions_[i];
                    v_des = v_des * hw_actuator_axis_directions_[i];
                    tau_ff = tau_ff * hw_actuator_axis_directions_[i];

                    // Wrap position to the range [-hw_actuator_position_scales_[i], hw_actuator_position_scales_[i]] to account for multiple rotations
                    p_des = wrap_angle(p_des, -hw_actuator_position_scales_[i], hw_actuator_position_scales_[i]);


                    // constrain commands to the permissible values for the CAN protocol
                    p_des = fminf(fmaxf(-hw_actuator_position_scales_[i], p_des), hw_actuator_position_scales_[i]);
                    v_des = fminf(fmaxf(-hw_actuator_velocity_scales_[i], v_des), hw_actuator_velocity_scales_[i]);
                    tau_ff = fminf(fmaxf(-hw_actuator_effort_scales_[i], tau_ff), hw_actuator_effort_scales_[i]);
                    kp = fminf(fmaxf(0.0, kp), hw_actuator_kp_scales_[i]);
                    kd = fminf(fmaxf(0.0, kd), hw_actuator_kd_scales_[i]);

                    // convert doubles to unsigned ints
                    int p_int =
                        double_to_uint(p_des, -hw_actuator_position_scales_[i], hw_actuator_position_scales_[i], 16);
                    int v_int =
                        double_to_uint(v_des, -hw_actuator_velocity_scales_[i], hw_actuator_velocity_scales_[i], 12);
                    int kp_int =
                        double_to_uint(kp, 0.0, hw_actuator_kp_scales_[i], 12);
                    int kd_int =
                        double_to_uint(kd, 0.0, hw_actuator_kd_scales_[i], 12);
                    int t_int =
                        double_to_uint(tau_ff, -hw_actuator_effort_scales_[i], hw_actuator_effort_scales_[i], 12);

                    // pack ints into the can message
                    pi3hat_input_.tx_can[i].data[0] = p_int >> 8;
                    pi3hat_input_.tx_can[i].data[1] = p_int & 0xFF;
                    pi3hat_input_.tx_can[i].data[2] = v_int >> 4;
                    pi3hat_input_.tx_can[i].data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
                    pi3hat_input_.tx_can[i].data[4] = kp_int & 0xFF;
                    pi3hat_input_.tx_can[i].data[5] = kd_int >> 4;
                    pi3hat_input_.tx_can[i].data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
                    pi3hat_input_.tx_can[i].data[7] = t_int & 0xff;
                    break;
                }
            default:
                RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to send: unknown CAN protocol");
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
            /* 
            The quaternion returned by the pi3hat is the rotation from the gravity frame to the IMU frame.
                Gravity frame:
                    +x and +y are parallel to the ground
                    +z points towards the ground
                IMU frame:
                    +x points towards the side of the Pi with the USB-C port
                    +y points towards the side of the Pi with the USB-A ports
                    +z points up from the Pi
            However, we want the rotation from the world frame to the IMU frame.
                World frame:
                    +x and +y are parallel to the ground
                    +z points towards the sky
            Therefore, we need to rotate the quaternion returned by the pi3hat by 180 degrees about the x-axis or y-axis. We choose to rotate about the x-axis.
            Let the quaternion returned by the pi3hat be (x, y, z, w).
            After applying a 180-degree rotation about the x-axis, the new quaternion is:
                (w, -z, y, -x)
            */
            hw_state_imu_orientation_[0] = pi3hat_input_.attitude->attitude.w; // x-component of the new quaternion
            hw_state_imu_orientation_[1] = -pi3hat_input_.attitude->attitude.z; // y-component of the new quaternion
            hw_state_imu_orientation_[2] = pi3hat_input_.attitude->attitude.y; // z-component of the new quaternion
            hw_state_imu_orientation_[3] = -pi3hat_input_.attitude->attitude.x; // w-component of the new quaternion
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
                // i is the index of the actuator in the hardware interface
                for (auto j = 0u; j < result.rx_can_size; j++)
                {
                    // j is the index of the can frame in the pi3hat input
                    switch (hw_actuator_can_protocols_[i])
                    {
                    case CanProtocol::CHEETAH:
                        {
                            int can_id = pi3hat_input_.rx_can[j].data[0];
                            if (pi3hat_input_.rx_can[j].bus == hw_actuator_can_channels_[i] && can_id == hw_actuator_can_ids_[i])
                            {
                                // parse the can frame
                                int p_int = (pi3hat_input_.rx_can[j].data[1] << 8) | pi3hat_input_.rx_can[j].data[2];
                                int v_int = (pi3hat_input_.rx_can[j].data[3] << 4) | (pi3hat_input_.rx_can[j].data[4] >> 4);
                                int i_int = ((pi3hat_input_.rx_can[j].data[4] & 0xF) << 8) | pi3hat_input_.rx_can[j].data[5];

                                // convert unsigned ints to doubles
                                double p_double = uint_to_double(p_int, -hw_actuator_position_scales_[i], hw_actuator_position_scales_[i], 16) * hw_actuator_axis_directions_[i] + hw_actuator_position_offsets_[i];
                                hw_state_velocities_[i] = uint_to_double(v_int, -hw_actuator_velocity_scales_[i], hw_actuator_velocity_scales_[i], 12) * hw_actuator_axis_directions_[i];
                                hw_state_efforts_[i] = uint_to_double(i_int, -hw_actuator_effort_scales_[i], hw_actuator_effort_scales_[i], 12) * hw_actuator_axis_directions_[i];

                                // position is wrapped to the range [-hw_actuator_position_scales_[i], hw_actuator_position_scales_[i]], so we need to unwrap it
                                hw_state_positions_[i] = unwrap_angle(p_double, hw_state_positions_[i], -hw_actuator_position_scales_[i], hw_actuator_position_scales_[i]);
                            }
                            break;
                        }                            
                    default:
                        RCLCPP_ERROR(rclcpp::get_logger("Pi3HatHardwareInterface"), "Failed to receive: unknown CAN protocol");
                        break;
                    }
                }
            }
        }
        // else
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("Pi3HatHardwareInterface"), "No CAN frames received");
        // }

        return hardware_interface::return_type::OK;
    }

    int Pi3HatHardwareInterface::double_to_uint(double x, double x_min, double x_max,
                      int bits)
    {
        /// Converts a double to an unsigned int, given range and number of bits ///
        double span = x_max - x_min;
        double offset = x_min;
        return (int)((x - offset) * ((double)((1 << bits) - 1)) / span);
    }

    double Pi3HatHardwareInterface::uint_to_double(int x_int, double x_min, double x_max,
                        int bits)
    {
        /// converts unsigned int to double, given range and number of bits ///
        double span = x_max - x_min;
        double offset = x_min;
        return ((double)x_int) * span / ((double)((1 << bits) - 1)) + offset;
    }

    double Pi3HatHardwareInterface::wrap_angle(double angle, double angle_min, double angle_max)
    {
        /// Wraps an angle to the range [angle_min, angle_max] ///
        double span = angle_max - angle_min;
        return angle - span * floor((angle - angle_min) / span);
    }

    double Pi3HatHardwareInterface::unwrap_angle(double angle, double prev_angle, double angle_min, double angle_max)
    {
        /// Deals with wrap-around for a continuously changing angle ///
        double span = angle_max - angle_min;
        double prev_angle_wrapped = wrap_angle(prev_angle, angle_min, angle_max);
        double d_angle = angle - prev_angle_wrapped;
        if (d_angle > span / 2)
        {
            d_angle -= span;
        }
        else if (d_angle < -span / 2)
        {
            d_angle += span;
        }
        return prev_angle + d_angle;
    }

} // namespace pi3hat_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    pi3hat_hardware_interface::Pi3HatHardwareInterface, hardware_interface::SystemInterface)