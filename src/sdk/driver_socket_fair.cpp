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

******************************************************************/
#include "whi_arm_interface/driver_socket_fair.h"
#include <json/json.h>

#include <thread>

DriverSocketFair::DriverSocketFair(const std::string& JointName, const std::string& Addr, int Port)
	: DriverBase(JointName), addr_(Addr), port_(Port)
{
	connector_ = std::make_unique<sockpp::tcp_connector>();
	connector_->connect(sockpp::inet_address(addr_, port_));
	connector_->read_timeout(std::chrono::seconds(5));
}

DriverSocketFair::~DriverSocketFair()
{
	close();
}

double DriverSocketFair::readAngle()
{
	return 0.0;
}

void DriverSocketFair::actuate(double Command)
{
}

void DriverSocketFair::actuate(std::string Command)
{
	sendCommand(Command);
	readFeedback();
#ifdef DEBUG
	std::cout << Command << std::endl;
#endif
}

void DriverSocketFair::close()
{
	connector_->close();
}

void DriverSocketFair::cal_angularVel2PwmDuty()
{
	// leave for override
}

void DriverSocketFair::set_debug_params(const std::map<std::string, bool>& DebugParams)
{
	// leave for override
	if (auto search = DebugParams.find("print_tcp_feedback"); search != DebugParams.end())
    {
		print_tcp_feedback_ = search->second;
	}
}

std::vector<std::string> DriverSocketFair::request(const std::vector<std::string>& Params)
{
	const std::string key("delay:");

	std::vector<std::string> res;

	for (int i = 0; i < Params.size(); ++i)
	{
		auto pos = Params[i].find(key);
		if (pos != std::string::npos)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(std::stoi(Params[i].substr(pos + key.length()))));
		}
		else
		{
			if (sendCommand(Params[i]))
			{
				std::string feedback;
    			const std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
				auto check = start;
				while (feedback.empty() && (check - start) < std::chrono::milliseconds(2000))
				{
					feedback = readFeedback();
					check = std::chrono::steady_clock::now();
				}
				res.push_back(feedback);
			}
			else
			{
				res.push_back(std::string());
				ROS_ERROR_STREAM("failed to send command " << Params[i]);
			}
		}
	}

	return res;
}

bool DriverSocketFair::sendCommand(const std::string& Command)
{
	if (connector_->is_open())
	{
		return connector_->write(Command).value() == Command.length();
	}
	else
	{
		ROS_FATAL_STREAM("failed to open socket " << addr_  << ":" << port_);
		return false;
	}
}

std::string DriverSocketFair::readFeedback()
{
	if (connector_->is_open())
	{
		uint8_t read[512] = { 0 };
		auto rc = connector_->read(read, sizeof(read));
		if (rc.value() > 0)
		{
			std::string feedback;
			feedback.assign((char*)read);
			if (print_tcp_feedback_)
			{
				std::cout << "DriverSocketFair::readFeedback: " << feedback << std::endl;
			}

			return feedback;
		}
	}
	else
	{
		ROS_FATAL_STREAM("failed to open socket " << addr_  << ":" << port_);
	}

	return std::string();
}
