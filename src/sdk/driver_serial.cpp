/******************************************************************
dc-motor driver instance for serial module

Features:
- dc-motor operation logic for serial hardware
- xxx

Prerequisites:
- sudo apt install ros-<ros distro>-serial

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/driver_serial.h"

DriverSerial::DriverSerial(const std::string& JointName, const char* Device, unsigned int Baudrate/* = 9600*/)
	: DriverBase(JointName)
	, serial_port_(Device), baudrate_(Baudrate)
{
	// serial
	try
	{
		serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
	}
	catch (serial::IOException& e)
	{
		ROS_FATAL_STREAM("failed to open serial " << serial_port_);
	}
}

DriverSerial::DriverSerial(const std::string& JointName, std::shared_ptr<serial::Serial> Serial)
	: DriverBase(JointName)
	, serial_inst_(Serial) {}

DriverSerial::~DriverSerial()
{
	close();
}

double DriverSerial::readAngle()
{
	return angular_value_;
}

void DriverSerial::actuate(double Command)
{
	drive(Command);
}

void DriverSerial::actuate(std::string Command)
{
	if (serial_inst_)
	{
		serial_inst_->write(Command);
	}
}

void DriverSerial::close()
{
	terminated_.store(true);
	th_read_.join();

	if (serial_inst_)
	{
		serial_inst_->close();
	}
}

void DriverSerial::cal_angularVel2PwmDuty()
{
	// leave for override
}

void DriverSerial::setMotor(const std::vector<int>& LimitsDir)
{
	if (serial_inst_)
	{
		std::string dirs("lt");
		for (const auto& it : LimitsDir)
		{
			dirs.append(it > 0 ? "1" : "0");
		}
		serial_inst_->write(dirs);

		// spawn the read thread
		th_read_ = std::thread(std::bind(&DriverSerial::threadReadSerial, this));
	}
}

void DriverSerial::setEncoder(unsigned int Resolution)
{

}

void DriverSerial::fetchData(unsigned char* Data, size_t Length)
{

}

void DriverSerial::drive(double Command)
{

}

void DriverSerial::resetEnc()
{

}

void DriverSerial::threadReadSerial()
{
	while (!terminated_.load())
	{
		size_t count = serial_inst_->available();
		if (count > 0)
		{
			unsigned char rbuff[count];
			size_t readNum = serial_inst_->read(rbuff, count);
			for (size_t i = 0; i < std::min(count, readNum); ++i)
			{
				std::cout << rbuff[i];
			}
			std::cout << std::endl;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
