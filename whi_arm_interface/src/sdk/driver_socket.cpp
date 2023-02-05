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
		std::vector<uint8_t> coded = codingCommand("SHUT");
		int res = connector_->write_n(coded.data(), coded.size());
		usleep(10000);
	}
}

double DriverSocket::readAngle()
{
	if (connector_->is_connected())
	{
		std::vector<uint8_t> coded = codingCommand("ANGLES");
		int res = connector_->write_n(coded.data(), coded.size());
		uint8_t read[4096] = { 0 };
		ssize_t readCount = connector_->read_n(read, sizeof(read));
		if (readCount > 0)
		{
			for (const auto& it : read)
			{
				std::cout << std::to_string(it) << ",";
			}
			std::cout << std::endl;
		}
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
#ifdef DEBUG
		std::string Command("MOVEJ,1,-123.456,30,90");
		std::vector<uint8_t> coded = codingCommand(Command);
		std::cout << "debug coding with length: " << coded.size() << std::endl;
		for (const auto& it : coded)
		{
			std::cout << std::hex << int(it) << ",";
		}
		std::cout << std::endl;
#endif
		std::vector<uint8_t> coded = codingCommand("SERVO");
		int res = connector_->write_n(coded.data(), coded.size());
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
		std::vector<uint8_t> coded = codingCommand("RUN_STATE");
		int res = connector_->write_n(coded.data(), coded.size());
	}

	return 0;
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
