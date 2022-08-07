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

Changelog:
2022-07-10: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "driver_base.h"

#include <std_msgs/String.h>
#include <functional>

class DriverRosserial : public DriverBase
{
public:
	using ResponseFunc = std::function<void(const std::string& State)>;

public:
	DriverRosserial() = delete;
	DriverRosserial(const std::string& JointName, std::shared_ptr<ros::NodeHandle> NodeHandle, std::string Topic);
	~DriverRosserial() override;

public:
	// override
	double readAngle() override;
	void actuate(double Command) override;
	void actuate(std::string Command) override;
	std::shared_ptr<RotaryEncoderBase> getEncoder() override { return encoder_; };
	void cal_angularVel2PwmDuty() override;

public:
	// specific
	void setMotor(int ForwardDir);
	void setEncoder(unsigned int Resolution);
	void registerResponse(ResponseFunc Func);

protected:
	void resetEnc();
	void callbackResponseMsg(const std_msgs::String::ConstPtr& Msg);

protected:
	std::unique_ptr<ros::Publisher> pub_{ nullptr };
	std::unique_ptr<ros::Subscriber> sub_{ nullptr };
	int forward_dir_{ 1 };
	double angular_value_;
	std::shared_ptr<RotaryEncoderBase> encoder_{ nullptr };
	long pre_encoder_{ 0 };
	ResponseFunc responseCallback_{ nullptr };
};
