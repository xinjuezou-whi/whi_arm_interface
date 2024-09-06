/******************************************************************
dc-motor driver instance for socket module

Features:
- dc-motor operation logic for socket hardware
- xxx

Dependency:
- sockpp, https://github.com/fpagliughi/sockpp
- CRC++, https://github.com/d-bahr/CRCpp
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-12-15: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "driver_base.h"
#include "sockpp/tcp_connector.h"

class DriverSocket : public DriverBase
{
public:
	DriverSocket() = delete;
	DriverSocket(const std::string& JointName);
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
	void request(const std::vector<std::string>& Params);
	std::vector<double> readParam(std::string& Param);
	int getState();
	bool isServoOn(uint32_t Duration = 500) const;
	bool sendCommand(const std::string& Command);
	std::vector<std::string> readFeedback();
	std::vector<uint8_t> codingCommand(const std::string& Command) const;
	std::vector<std::string> decodingFeedback(const uint8_t* Data, size_t Length) const;

protected:
	std::unique_ptr<sockpp::tcp_connector> connector_{ nullptr };
	double angular_value_{ 0.0 };
	std::map<std::string, std::vector<double>> response_;
	std::vector<double> angles_;
	std::vector<double> velocities_;
	std::shared_ptr<RotaryEncoderBase> encoder_{ nullptr };
	std::string addr_;
	int port_{ 8888 };
	std::vector<uint8_t> coded_command_;
	std::unique_ptr<uint64_t> tick_servo_{ nullptr };
};
