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
	if (connector_->is_connected())
	{
		sendCommand("SHUT");
		usleep(10000);
	}
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
		bool servoOn = false;
		std::string cmd("SERVO_STATE");
		if (sendCommand(cmd))
		{
			std::vector<std::string> feedback = readFeedback(cmd);
			for (const auto& it : feedback)
			{
				servoOn |= (it != "OFF");
			}
		}
		if (!servoOn)
		{
			std::vector<uint8_t> coded = codingCommand("SERVO");
			int res = connector_->write_n(coded.data(), coded.size());
		}
	}
	else
	{
		ROS_FATAL_STREAM_NAMED("failed to open socket %s", (addr_ + ":" + std::to_string(port_)).c_str());
	}
}

std::vector<double> DriverSocket::readAngles()
{
	std::vector<double> angles;

	if (connector_->is_connected())
	{
		std::string cmd("ANGLES");
		if (sendCommand(cmd))
		{
			std::vector<std::string> feedback = readFeedback(cmd);
			for (const auto& it : feedback)
			{
				angles.push_back(std::stod(it));
			}
		}
	}

	return angles;
}

int DriverSocket::getState()
{
	if (connector_->is_connected())
	{
		std::string cmd("RUN_STATE");
		if (sendCommand(cmd))
		{
			std::vector<std::string> feedback = readFeedback(cmd);
			for (const auto& it : feedback)
			{
				std::cout << it << std::endl;
			}
		}
	}

	return 0;
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
	std::vector<uint8_t> coded = codingCommand(Command);
	return connector_->write_n(coded.data(), coded.size()) == coded.size();
}

std::vector<std::string> DriverSocket::readFeedback(const std::string& Command)
{
	std::vector<std::string> data;

	uint8_t read[128] = { 0 };
	ssize_t readCount = connector_->read(read, sizeof(read));
	if (readCount > 0)
	{
		auto feedback = decodingFeedback(read);
		if (feedback.front() == Command)
		{
			for (size_t i = 1; i < feedback.size(); ++i)
			{
				data.push_back(feedback[i]);
			}
		}
	}

	return data;
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

std::vector<std::string> DriverSocket::decodingFeedback(const uint8_t* Data) const
{
#ifdef DEBUG
	for (const auto& it : Data)
	{
		std::cout << std::to_string(it) << ",";
	}
	std::cout << std::endl;
#endif
	size_t dataLength = (uint8_t(Data[0] << 24) | uint8_t(Data[1] << 16) | uint8_t(Data[2] << 8) | Data[3]) - 4;
	uint32_t crc = uint32_t(Data[dataLength + 4] << 24) | (Data[dataLength + 5] << 16) |
		(Data[dataLength + 6] << 8) | Data[dataLength + 7];
	std::string feedback((char*)Data + 4, dataLength);
	std::uint32_t readCrc = CRC::Calculate(feedback.c_str(), feedback.length(), CRC::CRC_32());
	if (crc == readCrc)
	{
		std::cout << "feedback " << feedback << std::endl;
		return split(feedback, ",");
	}

	return std::vector<std::string>();
}
