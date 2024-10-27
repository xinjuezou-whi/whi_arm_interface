/******************************************************************
dc-motor driver instance for socket module

Features:
- dc-motor operation logic for socket hardware
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

class DriverSocketJson : public DriverBase
{
public:
	enum ParamType { STR = 0, NUM };

public:
	DriverSocketJson() = delete;
	DriverSocketJson(const std::string& JointName, const std::string& Addr, int Port);
	~DriverSocketJson() override;

public:
	// override
	double readAngle() override;
	void actuate(double Command) override;
	void actuate(std::string Command) override;
	void close() override;
	std::shared_ptr<RotaryEncoderBase> getEncoder() override { return encoder_; };
	void cal_angularVel2PwmDuty() override;

public:
	// specific
	std::vector<int> request(const std::vector<std::string>& Params);
	std::vector<double> readParam(const std::string& Param);
	std::vector<std::string> readParamStr(const std::string& Param);
	bool sendCommand(const std::string& Command);
	std::string readFeedback();
	std::vector<uint8_t> codingCommand(const std::string& Command) const;
	void setParamsKey(const char*const* Keys, int Size);

protected:
	std::unique_ptr<sockpp::tcp_connector> connector_{ nullptr };
	std::map<std::string, std::vector<double>> response_;
	std::map<std::string, std::vector<std::string>> response_str_;
	std::shared_ptr<RotaryEncoderBase> encoder_{ nullptr };
	std::string addr_;
	int port_{ 8888 };
	std::vector<std::string> params_key_;
};
