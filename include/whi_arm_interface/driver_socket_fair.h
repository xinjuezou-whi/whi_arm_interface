/******************************************************************
dc-motor driver instance for socket module

Features:
- dc-motor operation logic for socket hardware: specific FAIR arm
- xxx

Dependency:
- sockpp, https://github.com/fpagliughi/sockpp
- jsoncpp, https://github.com/open-source-parsers/jsoncpp
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-09-04: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "driver_base.h"
#include "sockpp/tcp_connector.h"

class DriverSocketFair : public DriverBase
{
public:
	enum ParamType { STR = 0, NUM };

public:
	DriverSocketFair() = delete;
	DriverSocketFair(const std::string& JointName, const std::string& Addr, int Port);
	~DriverSocketFair() override;

public:
	// override
	double readAngle() override;
	void actuate(double Command) override;
	void actuate(std::string Command) override;
	void close() override;
	std::shared_ptr<RotaryEncoderBase> getEncoder() override { return encoder_; };
	void cal_angularVel2PwmDuty() override;
	void set_debug_params(const std::map<std::string, bool>& DebugParams) override;

public:
	// specific
	std::vector<std::string> request(const std::vector<std::string>& Params);
	bool sendCommand(const std::string& Command);
	std::string readFeedback();

protected:
	std::unique_ptr<sockpp::tcp_connector> connector_{ nullptr };
	std::shared_ptr<RotaryEncoderBase> encoder_{ nullptr };
	std::string addr_;
	int port_{ 8080 };
	// debug params
	bool print_tcp_feedback_{ false };
};
