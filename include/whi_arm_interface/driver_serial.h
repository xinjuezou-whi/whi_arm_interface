/******************************************************************
dc-motor driver instance for serial module

Features:
- dc-motor operation logic for serial hardware
- xxx

Prerequisites:
- sudo apt install ros-<ros distro>-serial

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-07-10: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "driver_base.h"

#include <serial/serial.h>
#include <mutex>
#include <thread>

// forward declare
class RotaryEncoderSerial;

class DriverSerial : public DriverBase
{
public:
	DriverSerial() = delete;
	DriverSerial(const std::string& JointName, const char* Device, unsigned int Baudrate = 9600);
	DriverSerial(const std::string& JointName, std::shared_ptr<serial::Serial> Serial);
	~DriverSerial() override;

public:
	// override
	double readAngle() override;
	void actuate(double Command) override;
	void actuate(std::string Command) override;
	std::shared_ptr<RotaryEncoderBase> getEncoder() override { return encoder_; };
	void cal_angularVel2PwmDuty() override;

public:
	// specific
	void setMotor(const std::vector<int>& LimitsDir);
	void setEncoder(unsigned int Resolution);

protected:
	void fetchData(unsigned char* Data, size_t Length);
	void drive(double Command);
	void resetEnc();
	void threadReadSerial();

protected:
	std::string serial_port_{ "" };
	unsigned int baudrate_{ 9600 };
	std::shared_ptr<serial::Serial> serial_inst_{ nullptr };
	std::mutex mtx_;
	double angular_value_;
	std::shared_ptr<RotaryEncoderBase> encoder_{ nullptr };
	std::thread th_read_;
	std::atomic_bool terminated_{ false };
	long pre_encoder_{ 0 };
};
