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
#include <json/json.h>

#include <thread>

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
	return 0.0;
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
	const std::string key("delay:");

	for (int i = 0; i < Params.size(); )
	{
		auto pos = Params[i].find(key);
		if (pos != std::string::npos)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(std::stoi(Params[i].substr(pos + key.length()))));
			++i;
		}
		else
		{
			if (sendCommand(Params[i]))
			{
				if (!readFeedback().empty())
				{
					++i;
				}
			}
			else
			{
				++i;
				ROS_ERROR_STREAM("failed to send command " << Params[i]);
			}
		}
	}
}

std::vector<double> DriverSocketJson::readParam(const std::string& Param)
{
	return response_[Param];
}

bool DriverSocketJson::sendCommand(const std::string& Command)
{
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

std::string DriverSocketJson::readFeedback()
{
	if (connector_->is_open())
	{
		uint8_t read[256] = { 0 };
		auto rc = connector_->read(read, sizeof(read));
		if (rc.value() > 0)
		{
			std::string feedback;
			feedback.assign((char*)read);
#ifdef DEBUG
			std::cout << "read feedback " << feedback << std::endl;
#endif

			for (const auto& key : params_key_)
			{
				if (feedback.find(key) != std::string::npos)
				{
					const auto rawJsonLength = static_cast<int>(feedback.length());
					Json::CharReaderBuilder builder;
					const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
					Json::Value root;
					JSONCPP_STRING err;
					reader->parse(feedback.c_str(), feedback.c_str() + rawJsonLength, &root, &err);

					const Json::Value joint_pos = root[key];
					std::vector<double> values;
					for (const auto& it : joint_pos)
					{
						values.push_back(it.asDouble());
					}
					response_[key] = values;
#ifdef DEBUG
					std::cout << "read param with key " << key << ":";
					for (const auto& it : response_[key])
					{
						std::cout << it << ",";
					}
					std::cout << std::endl;
#endif
				}
			}

			return feedback;
		}
	}
	else
	{
		ROS_FATAL_STREAM_NAMED("failed to open socket %s", (addr_ + ":" + std::to_string(port_)).c_str());
	}

	return std::string();
}

std::vector<uint8_t> DriverSocketJson::codingCommand(const std::string& Command) const
{
	std::vector<uint8_t> coded;
	for (const auto& it : Command)
	{
		coded.push_back(it);
	}

	return coded;
}

void DriverSocketJson::setParamsKey(const char*const* Keys, int Size)
{
	for (int i = 0; i < Size; ++i)
	{
		params_key_.push_back(Keys[i]);
	}
}
