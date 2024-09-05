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

******************************************************************/
#include "whi_arm_interface/driver_socket_json.h"

DriverSocketJson::DriverSocketJson(const std::string& JointName)
	: DriverBase(JointName)
{

}

DriverSocketJson::DriverSocketJson(const std::string& JointName, const std::string& Addr, int Port)
	: DriverBase(JointName), addr_(Addr), port_(Port)
{
	connector_ = std::make_unique<sockpp::tcp_connector>();
	connector_->connect(sockpp::inet_address(addr_, port_));
	connector_->read_timeout(std::chrono::seconds(5));
}

DriverSocketJson::~DriverSocketJson()
{
	connector_->close();
}

double DriverSocketJson::readAngle()
{
	return angular_value_;
}

void DriverSocketJson::actuate(double Command)
{
}

void DriverSocketJson::actuate(std::string Command)
{
	sendCommand(Command);
	readFeedback();
#ifdef DEBUG
	std::cout << Command << std::endl;
#endif
}

void DriverSocketJson::cal_angularVel2PwmDuty()
{
	// leave for override
}

void DriverSocketJson::request(const std::vector<std::string>& Params)
{
	for (const auto& req : Params)
	{
		if (sendCommand(req))
		{
			std::vector<std::string> feedback = readFeedback();
			if (!feedback.empty() && std::find(Params.begin(), Params.end(), feedback.front()) != Params.end())
			{
				std::vector<double> values;
				for (size_t i = 1; i < feedback.size(); ++i)
				{
					values.push_back(std::stod(feedback[i]));
				}
				response_[feedback.front()] = values;
			}
		}
	}
}

std::vector<double> DriverSocketJson::readParam(std::string& Param)
{
	return response_[Param];
}

int DriverSocketJson::getState()
{
	std::string cmd("RUN_STATE");
	if (sendCommand(cmd))
	{
		std::vector<std::string> feedback = readFeedback();
		if (!feedback.empty() && cmd == feedback.front())
		{
			for (size_t i = 1; i < feedback.size(); ++i)
			{
				std::cout << feedback[i] << std::endl;
			}
		}
	}

	return 0;
}

bool DriverSocketJson::isServoOn(uint32_t Duration/* = 500*/) const
{
	if (tick_servo_)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool DriverSocketJson::sendCommand(const std::string& Command)
{
#ifdef DEBUG
	std::string Command("MOVEJ,1,-123.456,30,90");
	std::vector<uint8_t> codedDebug = codingCommand(Command);
	std::cout << "debug coding with length: " << codedDebug.size() << std::endl;
	for (const auto& it : codedDebug)
	{
		std::cout << std::hex << int(it) << ",";
	}
	std::cout << std::endl;
#endif
	if (connector_->is_open())
	{
		std::vector<uint8_t> coded = codingCommand(Command);
		return connector_->write_n(coded.data(), coded.size()).value() == coded.size();
	}
	else
	{
		ROS_FATAL_STREAM_NAMED("failed to open socket %s", (addr_ + ":" + std::to_string(port_)).c_str());
		return false;
	}
}

std::vector<std::string> DriverSocketJson::readFeedback()
{
	uint8_t read[1] = { 0 };
	auto rc = connector_->read(read, sizeof(read));
	if (rc.value() > 0)
	{
		return decodingFeedback(read, sizeof(read));
	}

	return std::vector<std::string>();
}

std::vector<uint8_t> DriverSocketJson::codingCommand(const std::string& Command) const
{
	uint32_t length;
	std::vector<uint8_t> coded;
#ifdef DEBUG
	std::cout << "len " << length << " " << std::bitset<8>{ length } << std::endl;
#endif
	for (int i = int(sizeof(length)) - 1; i >= 0 ; --i)
	{
		coded.push_back(uint8_t(length >> (i * 8)));
	}
	for (const auto& it : Command)
	{
		coded.push_back(it);
	}

	return coded;
}

std::vector<std::string> DriverSocketJson::decodingFeedback(const uint8_t* Data, size_t Length) const
{
#ifdef DEBUG
	for (size_t i = 0; i < Length; ++i)
	{
		std::cout << std::to_string(Data[i]) << ",";
	}
	std::cout << std::endl;
#endif
	size_t dataLength = (uint8_t(Data[0] << 24) | uint8_t(Data[1] << 16) | uint8_t(Data[2] << 8) | Data[3]) - 4;
	if (dataLength < Length)
	{
		uint32_t crc = uint32_t(Data[dataLength + 4] << 24) | (Data[dataLength + 5] << 16) |
			(Data[dataLength + 6] << 8) | Data[dataLength + 7];
		std::string feedback((char*)Data + 4, dataLength);
		std::uint32_t readCrc;
		if (crc == readCrc)
		{
#ifdef DEBUG
			std::cout << "feedback:" << feedback << std::endl;
#endif
			return std::vector<std::string>();
		}
	}

	return std::vector<std::string>();
}
