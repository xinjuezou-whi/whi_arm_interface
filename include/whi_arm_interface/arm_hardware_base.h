/******************************************************************
base arm hardware under ROS 2
it is a hardware resource layer for ros2_controller

Features:
- abstract arm hardware
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
202-06-25: Initial version
202-xx-xx: xxx
******************************************************************/
#pragma once
#include "driver_base.h"
#include <whi_interfaces/msg/whi_state.hpp>
#include <whi_interfaces/msg/whi_motion_state.hpp>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <string>
#include <memory>
#include <vector>

namespace whi_arm_interface
{
    // forward declaration
    class WhiArmInterface;

    /// Hardware base for a robot
    class ArmHardware
    {
	public:
		enum Hardware { I2C = 0, CAN_BUS, SERIAL, SOCKET, API, HARDWARE_SUM };
		static constexpr const char* hardware[HARDWARE_SUM] = { "i2c", "canbus", "serial", "socket", "api" };

    public:
        ArmHardware() = delete;
        ArmHardware(const std::string& Config, rclcpp::Node::SharedPtr Node)
            : node_handle_(Node)
        {
            std::string nodeName = node_handle_->get_name();
            pub_state_ = node_handle_->create_publisher<whi_interfaces::msg::WhiState>("whi_state", 10);
            pub_motion_state_ = node_handle_->create_publisher<whi_interfaces::msg::WhiMotionState>(nodeName + "_arm_state", 10);
            using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
            trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(node_handle_,
                "/motion_controller/follow_joint_trajectory"); // TODO: verify the action name
        };
        virtual ~ArmHardware() = default; 

    public:
        virtual std::string getSwEstopTopic() const = 0;
        virtual bool setIo(int Addr, int Level) = 0;
        virtual void read(WhiArmInterface* HwIf, double Dt) = 0;
        virtual void write(WhiArmInterface* HwIf, double Dt) = 0;
        virtual void quit() = 0;
        bool isInitialized () const
        {
            return initialized_;
        };
        bool isStandby() const
        {
            return standby_;
        };

    protected:
        virtual bool parseConfig(const std::string& Config) = 0;
        void publishState(int Level, const std::string& Key, const std::string& Value,
		    const std::string& Description = std::string(""))
        {
            whi_interfaces::msg::WhiState msg;
            msg.header.stamp = node_handle_->get_clock()->now();
            msg.hardware_id = "whi_arm_interface";
            msg.level = Level;
            msg.description = Description;
            diagnostic_msgs::msg::KeyValue value;
            value.key = Key;
            value.value = Value;
            msg.values.push_back(value);
        
            pub_state_->publish(msg);

            whi_interfaces::msg::WhiMotionState motionMsg;
            motionMsg.header.stamp = node_handle_->get_clock()->now();
            if (Value.find("standby") != std::string::npos)
            {
                motionMsg.state = whi_interfaces::msg::WhiMotionState::STA_STANDBY;
            }
            else if (Value.find("running") != std::string::npos)
            {
                motionMsg.state = whi_interfaces::msg::WhiMotionState::STA_RUNNING;
            }
            else if (Value.find("operating") != std::string::npos)
            {
                motionMsg.state = whi_interfaces::msg::WhiMotionState::STA_OPERATING;
            }
            else if (Value.find("estopped") != std::string::npos)
            {
                motionMsg.state = whi_interfaces::msg::WhiMotionState::STA_ESTOP;
            }
            else if (Value.find("protective") != std::string::npos)
            {
                motionMsg.state = whi_interfaces::msg::WhiMotionState::STA_FAULT;
            }
            else
            {
                motionMsg.state = whi_interfaces::msg::WhiMotionState::STA_FAULT;
            }
            pub_motion_state_->publish(motionMsg);
        }

    public:
		std::vector<double> joint_positions_;
		std::vector<double> joint_velocities_;
		std::vector<double> joint_efforts_;
		std::vector<double> joint_position_commands_;
		std::vector<double> joint_velocity_commands_;
        std::vector<double> joint_acceleration_commands_;
		std::vector<double> joint_effort_commands_;
        double speed_scaling_combined_{ 1.0 };

    protected:
        rclcpp::Node::SharedPtr node_handle_{ nullptr };
        rclcpp::Publisher<whi_interfaces::msg::WhiState>::SharedPtr pub_state_{ nullptr };
        rclcpp::Publisher<whi_interfaces::msg::WhiMotionState>::SharedPtr pub_motion_state_{ nullptr };
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_client_{ nullptr };

        std::map<std::string, std::unique_ptr<DriverBase>> drivers_map_;
		bool initialized_{ false };
        bool standby_{ false };
    };
} // namespace whi_arm_interface
