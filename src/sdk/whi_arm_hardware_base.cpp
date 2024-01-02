/******************************************************************
arm hardware interface under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- abstract arm hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/whi_arm_hardware_base.h"

namespace whi_arm_hardware_interface
{
	const char* ArmHardware::hardware[HARDWARE_SUM] = { "i2c", "canbus", "serial", "rosserial", "socket" };

	ArmHardware::ArmHardware(std::shared_ptr<ros::NodeHandle>& NodeHandle)
		: node_handle_(NodeHandle) {}

	ArmHardware::~ArmHardware()
	{
	}
}
