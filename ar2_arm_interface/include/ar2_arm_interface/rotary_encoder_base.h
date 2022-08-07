/******************************************************************
rotary encoder base for abstract interface

Features:
- abstract rotary encoder operation interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-03-16: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <string>
#include <atomic>

#define ROS_1 1
#define ROS_2 0
#if ROS_1
#include <ros/ros.h>
#elif ROS_2
#endif

class RotaryEncoderBase
{
public:
	RotaryEncoderBase() = delete;
	RotaryEncoderBase(const std::string& Name) : name_(Name) {};
	~RotaryEncoderBase() = default;

public:
	virtual long currentValue() = 0;
	virtual void reset() = 0;
	virtual unsigned int getResolution() = 0;
	virtual void enablePrintOut(bool Enable) { print_out_.store(Enable); };
	bool isPrintOutEnabled() { return print_out_.load(); };

protected:
	std::string name_;
	std::atomic_bool print_out_{ false };
};
