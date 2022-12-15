/******************************************************************
dc-motor driver base for abstract interface

Features:
- abstract dc-motor operation interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-03-07: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "rotary_encoder_base.h"

#include <memory>
#include <string>

class DriverBase
{
public:
	DriverBase() = delete;
	DriverBase(const std::string& JointName) : joint_name_(JointName) {};
	virtual ~DriverBase() = default;

public:
	virtual double readAngle() = 0;
	virtual void actuate(double Command) = 0;
	virtual void actuate(std::string Command) = 0;
	virtual std::shared_ptr<RotaryEncoderBase> getEncoder() = 0;
	virtual void cal_angularVel2PwmDuty() = 0;

protected:
	std::string joint_name_;
};
