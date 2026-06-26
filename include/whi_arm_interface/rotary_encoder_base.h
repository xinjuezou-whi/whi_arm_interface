/******************************************************************
rotary encoder base for abstract interface

Features:
- abstract rotary encoder operation interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-03-16: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <string>
#include <atomic>

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
