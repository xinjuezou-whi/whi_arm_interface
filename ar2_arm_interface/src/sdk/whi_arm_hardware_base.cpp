/******************************************************************
arm hardware interface under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- abstract arm hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "ar2_arm_interface/whi_arm_hardware_base.h"
/// <summary>
/// 
/// </summary>
namespace whi_arm_hardware_interface
{
	const char* ArmHardware::hardware[HARDWARE_SUM] = { "i2c", "canbus", "serial", "rosserial"};

	ArmHardware::ArmHardware(std::shared_ptr<ros::NodeHandle>& NodeHandle)
		: node_handle_(NodeHandle) {}

	ArmHardware::~ArmHardware()
	{
	}
}
