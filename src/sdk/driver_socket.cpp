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

******************************************************************/
#include "whi_arm_interface/driver_socket.h"
#include "CRC.h"

#include <bitset>
#include <regex>

DriverSocket::DriverSocket(const std::string& JointName)
	: DriverBase(JointName)
{

}

DriverSocket::DriverSocket(const std::string& JointName, const std::string& Addr, int Port)
	: DriverBase(JointName), addr_(Addr), port_(Port)
{
	connector_ = std::make_unique<sockpp::tcp_connector>();
	connector_->connect(sockpp::inet_address(addr_, port_));
	connector_->read_timeout(std::chrono::seconds(5));
}

DriverSocket::~DriverSocket()
{
	connector_->close();
}

std::vector<std::string> split(const std::string SrcStr, const std::string RegexStr)
{
    std::regex regexz(RegexStr);
    return { std::sregex_token_iterator(SrcStr.begin(), SrcStr.end(), regexz, -1),
		std::sregex_token_iterator() };
}

double DriverSocket::readAngle()
{
	return angular_value_;
}

void DriverSocket::actuate(double Command)
{
}

void DriverSocket::actuate(std::string Command)
{
	sendCommand(Command);
	readFeedback();
#ifdef DEBUG
	std::cout << Command << std::endl;
#endif
}

void DriverSocket::cal_angularVel2PwmDuty()
{
	// leave for override
}

uint64_t currentTick()
{
	return (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
}

void DriverSocket::setMotor(uint32_t ResponseLength)
{
	response_length_ = ResponseLength;

	bool servoOn = false;
	std::string cmd("SERVO_STATE");
	if (sendCommand(cmd))
	{
		std::vector<std::string> feedback = readFeedback();
		if (!feedback.empty() && cmd == feedback.front())
		{
			for (size_t i = 1; i < feedback.size(); ++i)
			{
				servoOn |= (feedback[i] != "OFF");
			}
		}
	}
	if (!servoOn)
	{
		std::vector<uint8_t> coded = codingCommand("SERVO");
		auto rc = connector_->write_n(coded.data(), coded.size());
		// give a breathe to arm
		tick_servo_ = std::make_unique<uint64_t>(currentTick());
	}
	else
	{
		tick_servo_ = std::make_unique<uint64_t>(0);
	}
}

void DriverSocket::request(const std::vector<std::string>& Params)
{
	for (int i  = 0; i < Params.size(); )
	{
		if (sendCommand(Params[i]))
		{
			std::vector<std::string> feedback = readFeedback();
			if (!feedback.empty())
			{
				++i;

				if (std::find(Params.begin(), Params.end(), feedback.front()) != Params.end())
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
		else
		{
			++i;
			ROS_ERROR_STREAM("failed to send command " << Params[i]);
		}
	}
}

std::vector<double> DriverSocket::readParam(std::string& Param)
{
	return response_[Param];
}

int DriverSocket::getState()
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

bool DriverSocket::isServoOn(uint32_t Duration/* = 500*/) const
{
	if (tick_servo_)
	{
		return currentTick() - *tick_servo_ > Duration;
	}
	else
	{
		return true;
	}
}

bool DriverSocket::sendCommand(const std::string& Command)
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

std::vector<std::string> DriverSocket::readFeedback()
{
	if (connector_->is_open())
	{
		uint8_t read[response_length_] = { 0 };
		auto rc = connector_->read(read, sizeof(read));
		if (rc.value() > 0)
		{
			return decodingFeedback(read, sizeof(read));
		}
	}
	else
	{
		ROS_FATAL_STREAM_NAMED("failed to open socket %s", (addr_ + ":" + std::to_string(port_)).c_str());
	}

	return std::vector<std::string>();
}

std::vector<uint8_t> DriverSocket::codingCommand(const std::string& Command) const
{
	std::uint32_t crc = CRC::Calculate(Command.c_str(), Command.length(), CRC::CRC_32());
	uint32_t length = Command.length() + sizeof(crc);
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
	for (int i = int(sizeof(crc)) - 1; i >= 0 ; --i)
	{
		coded.push_back(uint8_t(crc >> (i * 8)));
	}

	return coded;
}

std::vector<std::string> DriverSocket::decodingFeedback(const uint8_t* Data, size_t Length) const
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
		std::uint32_t readCrc = CRC::Calculate(feedback.c_str(), feedback.length(), CRC::CRC_32());
		if (crc == readCrc)
		{
#ifdef DEBUG
			std::cout << "feedback:" << feedback << std::endl;
#endif
			return split(feedback, ",");
		}
	}

	return std::vector<std::string>();
}
