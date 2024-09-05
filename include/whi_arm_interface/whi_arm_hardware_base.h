/******************************************************************
arm hardware interface under ROS 1
it is a hardware resouces layer for ros_control

Features:
- abstract arm hardware interfaces
- xxx

Dependency:
- sudo apt install apt ros-<distro>-joint-trajectory-controller

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-06-16: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "driver_base.h"

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>

namespace whi_arm_hardware_interface
{
	/// brief Hardware interface for a robot
	class ArmHardware : public hardware_interface::RobotHW
	{
	public:
		enum Hardware { I2C = 0, CAN_BUS, SERIAL, ROSSERIAL, SOCKET, JAKA_API, HARDWARE_SUM };
		static constexpr const char* hardware[HARDWARE_SUM] = { "i2c", "canbus", "serial", "rosserial", "socket", "jaka_api" };

	public:
		ArmHardware() = delete;
		ArmHardware(std::shared_ptr<ros::NodeHandle>& NodeHandle) : node_handle_(NodeHandle) {};
		virtual ~ArmHardware() {};

	protected:
		std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
		std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
		ros::Duration elapsed_time_;
		double loop_hz_{ 10.0 };

		/// joint state interface
		hardware_interface::JointStateInterface joint_state_interface_;
		/// joint command interface: position
		hardware_interface::PositionJointInterface position_joint_interface_;
		/// joint command interface: position, velocity
		hardware_interface::PosVelJointInterface pos_vel_joint_interface_;
		/// joint command interface: position, velocity, and acceleration
		hardware_interface::PosVelAccJointInterface pos_vel_acc_joint_interface_;
		/// joint limit interfaces
		joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
		joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;

		/// shared memory
		std::size_t num_joints_{ 0 };
		std::vector<std::string> joint_names_;
		std::vector<int> joint_types_;
		std::vector<double> joint_position_;
		std::vector<double> joint_velocity_;
		std::vector<double> joint_effort_;
		std::vector<double> joint_position_command_;
		std::vector<double> joint_velocity_command_;
		std::vector<double> joint_acceleration_command_;
		std::vector<double> joint_effort_command_;
		std::vector<double> joint_lower_limits_;
		std::vector<double> joint_upper_limits_;
		std::vector<double> joint_effort_limits_;

		/// controller manager
		std::unique_ptr<controller_manager::ControllerManager> controller_manager_{ nullptr };

		// driver
		std::map<std::string, std::unique_ptr<DriverBase>> drivers_map_;
	};
}
