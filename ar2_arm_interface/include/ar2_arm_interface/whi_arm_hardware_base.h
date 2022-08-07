/******************************************************************
arm hardware interface under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- abstract arm hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
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
		enum Hardware { I2C = 0, CAN_BUS, SERIAL, ROSSERIAL, HARDWARE_SUM };
		static const char* hardware[HARDWARE_SUM];

	public:
		ArmHardware() = delete;
		ArmHardware(std::shared_ptr<ros::NodeHandle>& NodeHandle);
		virtual ~ArmHardware();

	protected:
		std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
		std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
		ros::Duration elapsed_time_;
		double loop_hz_{ 10.0 };

		/// interfaces
		hardware_interface::JointStateInterface joint_state_interface_;
		hardware_interface::PositionJointInterface position_joint_interface_;

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
