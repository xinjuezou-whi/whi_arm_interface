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

DriverSocket::DriverSocket(const std::string& JointName, std::shared_ptr<StreamSocket> Socket)
	: DriverBase(JointName), socket_(Socket)
{

}

DriverSocket::DriverSocket(const std::string& JointName, const std::string& Addr, int Port)
	: DriverBase(JointName), addr_(Addr), port_(Port)
{

}

DriverSocket::~DriverSocket()
{

}

double DriverSocket::readAngle()
{
	if (socket_)
	{
		std::string cmd("ANGLES\r\n");
		socket_->send(cmd.c_str(), cmd.length());
		socket_->waitforData();
		char data[256] = { 0 };
		size_t recvLen = 0;
		socket_->recv(data, sizeof(data), recvLen);
	}
	return angular_value_;
}

void DriverSocket::actuate(double Command)
{
}

void DriverSocket::actuate(std::string Command)
{
	if (socket_)
	{
		socket_->send(Command.c_str(), Command.length());
	}
}

void DriverSocket::cal_angularVel2PwmDuty()
{
	// leave for override
}

void DriverSocket::setMotor()
{
	if (!socket_)
	{
		socket_ = std::make_shared<StreamSocket>();
	}

	SocketAddress addrPair(addr_, port_);
	socket_->connect(addrPair);
	std::string cmd("SERVO\r\n");
	socket_->send(cmd.c_str(), cmd.length());
}

int DriverSocket::getState()
{
	if (socket_)
	{
		std::string cmd("RUN_STATE\r\n");
		socket_->send(cmd.c_str(), cmd.length());
		socket_->waitforData();
		char data[32] = { 0 };
		size_t recvLen = 0;
		socket_->recv(data, sizeof(data), recvLen);
	}

	return 0;
}
