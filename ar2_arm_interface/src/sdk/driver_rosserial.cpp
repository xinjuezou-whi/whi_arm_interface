/******************************************************************
dc-motor driver instance for rosserial module

Features:
- dc-motor operation logic for rosserial hardware
- xxx

Prerequisites:
- sudo apt install ros-<ros distro>-rosserial
- sudo apt install ros-<ros distro>-rosserial-arduino

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "ar2_arm_interface/driver_rosserial.h"

DriverRosserial::DriverRosserial(const std::string& JointName, std::shared_ptr<ros::NodeHandle> NodeHandle, std::string Topic)
	: DriverBase(JointName)
	, pub_(std::make_unique<ros::Publisher>(NodeHandle->advertise<std_msgs::String>(Topic, 1)))
	, sub_(std::make_unique<ros::Subscriber>(NodeHandle->subscribe<std_msgs::String>("arm_hardware_response", 1,
		std::bind(&DriverRosserial::callbackResponseMsg, this, std::placeholders::_1)))) {}

DriverRosserial::~DriverRosserial()
{
	// TODO: publish the home position
}

double DriverRosserial::readAngle()
{
	return angular_value_;
}

void DriverRosserial::actuate(double Command)
{
}

void DriverRosserial::actuate(std::string Command)
{
	if (pub_)
	{
		std_msgs::String strMsg;
		strMsg.data = Command;
		pub_->publish(strMsg);
	}
}

void DriverRosserial::cal_angularVel2PwmDuty()
{
	// leave for override
}

void DriverRosserial::setMotor(int ForwardDir)
{
	forward_dir_ = ForwardDir;
}

void DriverRosserial::setEncoder(unsigned int Resolution)
{

}

void DriverRosserial::registerResponse(ResponseFunc Func)
{
	responseCallback_ = Func;
}

void DriverRosserial::resetEnc()
{

}

void DriverRosserial::callbackResponseMsg(const std_msgs::String::ConstPtr& Msg)
{
	std::string response(Msg->data);

	if (responseCallback_)
	{
		responseCallback_(Msg->data);
	}
}
