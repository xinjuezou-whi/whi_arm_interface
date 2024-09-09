/******************************************************************
node to handle arm hardwares
it is a hardware resouces layer for ros_controller

Features:
- hardware resouces setup logic
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-06-13: Initial version
2022-xx-xx: xxx
******************************************************************/
#include <signal.h>
#include <functional>
#include <iostream>

#include "whi_arm_interface/arm_hardware_ar.h"
#include "whi_arm_interface/arm_hardware_chin.h"
#include "whi_arm_interface/arm_hardware_jaka.h"

#define ASYNC 1

// since ctrl-c break cannot trigger destructor, override the signal interruption 
std::function<void(int)> functionWrapper;
void signalHandler(int Signal)
{
	functionWrapper(Signal);
}

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI arm interface VERSION 03.01.6" << std::endl;
	std::cout << "Copyright © 2022-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
	ros::init(argc, argv, "whi_arm_interface");
	auto nodeHandle = std::make_shared<ros::NodeHandle>();

	/// node logic
	ros::NodeHandle nhPrivate("~");
	std::string arm;
	nhPrivate.param("arm", arm, std::string("whi"));
	std::unique_ptr<whi_arm_hardware_interface::ArmHardware> armHardware = nullptr;
	if (arm == "ar")
	{
		armHardware = std::make_unique<whi_arm_hardware_interface::ArHardwareInterface>(nodeHandle);
	}
	else if (arm == "chin")
	{
		armHardware = std::make_unique<whi_arm_hardware_interface::ChinHardwareInterface>(nodeHandle);
	}
	else if (arm == "jaka")
	{
		armHardware = std::make_unique<whi_arm_hardware_interface::JakaHardwareInterface>(nodeHandle);
	}

	// override the default ros sigint handler, with this override the shutdown will be gracefull
	// NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		armHardware->quit();

		// all the default sigint handler does is call shutdown()
		ros::shutdown();
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::waitForShutdown();
#else
	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
#endif

	std::cout << "whi_arm_interface exited" << std::endl;

	return 0;
}
