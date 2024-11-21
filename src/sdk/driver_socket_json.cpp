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

DriverSocketJson::DriverSocketJson(const std::string& JointName, const std::string& Addr, int Port)
	: DriverBase(JointName), addr_(Addr), port_(Port)
{
	connector_ = std::make_unique<sockpp::tcp_connector>();
	connector_->connect(sockpp::inet_address(addr_, port_));
	connector_->read_timeout(std::chrono::seconds(5));
}

DriverSocketJson::~DriverSocketJson()
{
	close();
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

void DriverSocketJson::close()
{
	connector_->close();
}

void DriverSocketJson::cal_angularVel2PwmDuty()
{
	// leave for override
}

void DriverSocketJson::set_debug_params(const std::map<std::string, bool>& DebugParams)
{
	// leave for override
	if (auto search = DebugParams.find("print_tcp_feedback"); search != DebugParams.end())
    {
		print_tcp_feedback_ = search->second;
	}
}

std::vector<int> DriverSocketJson::request(const std::vector<std::string>& Params)
{
	const std::string key("delay:");

	std::vector<int> res;

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
				while (feedback.empty())
				{
					feedback = readFeedback();
				}
				const auto rawJsonLength = static_cast<int>(feedback.length());
				Json::CharReaderBuilder builder;
				const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
				Json::Value root;
				JSONCPP_STRING err;
				reader->parse(feedback.c_str(), feedback.c_str() + rawJsonLength, &root, &err);

				const Json::Value errorCode = root["errorCode"];
				if (errorCode.asString() != "0")
				{
					res.push_back(i);
				}
			}
			else
			{
				res.push_back(i);
				ROS_ERROR_STREAM("failed to send command " << Params[i]);
			}
		}
	}

	return res;
}

std::vector<double> DriverSocketJson::readParam(const std::string& Param)
{
	return response_[Param];
}

std::vector<std::string> DriverSocketJson::readParamStr(const std::string& Param)
{
	return response_str_[Param];
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
		ROS_FATAL_STREAM("failed to open socket " << addr_  << ":" << port_);
		return false;
	}
}

std::string DriverSocketJson::readFeedback()
{
	if (connector_->is_open())
	{
		uint8_t read[384] = { 0 };
		auto rc = connector_->read(read, sizeof(read));
		if (rc.value() > 0)
		{
			std::string feedback;
			feedback.assign((char*)read);
			if (print_tcp_feedback_)
			{
				std::cout << "DriverSocketJson::readFeedback: " << feedback << std::endl;
			}

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

					const Json::Value data = root[key];
					std::vector<double> values;
					std::vector<std::string> valuesStr;
					if (data.isArray())
					{
						for (const auto& it : data)
						{
							if (it.isNumeric())
							{
								values.push_back(it.asDouble());
							}
							else if (it.isString())
							{
								valuesStr.push_back(it.asString());
							}
							else if (it.isBool())
							{
								valuesStr.push_back(it.asBool() ? "1" : "0");
							}
						}
					}
					else
					{
						if (data.isNumeric())
						{
							values.push_back(data.asDouble());
						}
						else if (data.isString())
						{
							valuesStr.push_back(data.asString());
						}
						else if (data.isBool())
						{
							valuesStr.push_back(data.asBool() ? "1" : "0");
						}
					}
					if (!values.empty())
					{
						response_[key] = values;
					}
					else if (!valuesStr.empty())
					{
						response_str_[key] = valuesStr;
					}
#ifdef DEBUG
					std::cout << "read param with key " << key << ":";
					for (const auto& it : response_[key])
					{
						std::cout << it << ",";
					}
					for (const auto& it : response_str_[key])
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
		ROS_FATAL_STREAM("failed to open socket " << addr_  << ":" << port_);
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
