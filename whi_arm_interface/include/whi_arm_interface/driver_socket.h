/******************************************************************
dc-motor driver instance for socket module

Features:
- dc-motor operation logic for socket hardware
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-12-15: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "driver_base.h"
#include "net_socket.h"

class DriverSocket : public DriverBase
{
public:
	DriverSocket() = delete;
	DriverSocket(const std::string& JointName, std::shared_ptr<StreamSocket> Socket);
	DriverSocket(const std::string& JointName, const std::string& Addr, int Port);
	~DriverSocket() override;

public:
	// override
	double readAngle() override;
	void actuate(double Command) override;
	void actuate(std::string Command) override;
	std::shared_ptr<RotaryEncoderBase> getEncoder() override { return encoder_; };
	void cal_angularVel2PwmDuty() override;

public:
	// specific
	void setMotor();
	int getState();

protected:
	double angular_value_{ 0.0 };
	std::shared_ptr<RotaryEncoderBase> encoder_{ nullptr };
	std::shared_ptr<StreamSocket> socket_{ nullptr };
	std::string addr_;
	int port_{ 8888 };
};
