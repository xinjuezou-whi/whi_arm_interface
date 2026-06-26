/******************************************************************
arm hardware interface under ROS 2
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
2026-06-24: Migrate to ROS 2
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "arm_hardware_base.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

namespace whi_arm_hardware_interface
{
	/// brief Hardware interface for a robot
	class WhiArmInterface : public hardware_interface::SystemInterface
	{
	public:
        RCLCPP_SHARED_PTR_DEFINITIONS(WhiArmInterface);
    
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& Params) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& PreState) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& PreState) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& PreState) override;

		std::vector<hardware_interface::StateInterface> export_state_interfaces() final;
		std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

        hardware_interface::return_type read(const rclcpp::Time& Time,
            const rclcpp::Duration& Period) override;
        hardware_interface::return_type write(const rclcpp::Time& Time,
            const rclcpp::Duration& Period) override;

    private:
        // Parameters for the simulation
        double hw_start_seconds_{ 0.2 };
        double hw_stop_seconds_{ 1.0 };

	protected:
		std::unique_ptr<ArmHardware> hardware_{ nullptr };
	};
} // namespace whi_arm_hardware_interface
