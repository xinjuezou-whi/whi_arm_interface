/******************************************************************
dc-motor driver instance for socket module

Features:
- dc-motor operation logic for socket hardware
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/driver_socket.h"

DriverSocket::DriverSocket(const std::string& JointName)
	: DriverBase(JointName)
{

}

DriverSocket::DriverSocket(const std::string& JointName, const std::string& Addr, int Port)
	: DriverBase(JointName), addr_(Addr), port_(Port)
{
	connector_ = std::make_unique<sockpp::tcp_connector>();
	connector_->connect(sockpp::inet_address(addr_, port_));
}

DriverSocket::~DriverSocket()
{
	if (connector_->is_connected())
	{
		std::string cmd("SHUT\r\n");
		int res = connector_->write_n(cmd.c_str(), cmd.length());
	}
}

double DriverSocket::readAngle()
{
	if (connector_->is_connected())
	{
		std::string cmd("ANGLES\r\n");
		int res = connector_->write_n(cmd.c_str(), cmd.length());
	}
	return angular_value_;
}

void DriverSocket::actuate(double Command)
{
}

void DriverSocket::actuate(std::string Command)
{
	if (connector_->is_connected())
	{
		int res = connector_->write_n(Command.c_str(), Command.length());
	}
}

void DriverSocket::cal_angularVel2PwmDuty()
{
	// leave for override
}

void DriverSocket::setMotor()
{
	if (connector_->is_connected())
	{
		std::string cmd("SERVO\r\n");
		int res = connector_->write_n(cmd.c_str(), cmd.length());
	}
	else
	{
		ROS_FATAL_STREAM_NAMED("failed to open socket %s", (addr_ + ":" + std::to_string(port_)).c_str());
	}
}

int DriverSocket::getState()
{
	if (connector_->is_connected())
	{
		std::string cmd("RUN_STATE\r\n");
		int res = connector_->write_n(cmd.c_str(), cmd.length());
	}

	return 0;
}
