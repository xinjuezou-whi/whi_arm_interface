/******************************************************************
arm hardware interface of JAKA under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- JAKA hardware interfaces
- xxx

Dependency:
- libjakaAPI.so
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-09-03: Initial version
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include "arm_hardware_base.h"
#include "whi_interfaces/srv/whi_srv_io.hpp"
#include "fairAPI/robot.h"

#include <std_srvs/srv/trigger.hpp>

namespace whi_arm_interface
{
    // forward declaration
    class HwConfig;

    class FairHardwareInterface : public ArmHardware
    {
    public:
        FairHardwareInterface(const std::string& Config, rclcpp::Node::SharedPtr Node);
        virtual ~FairHardwareInterface();

    public:
        std::string getSwEstopTopic() const override;
        bool setIo(int Addr, int Level) override;
        void read(WhiArmInterface* HwIf, double Dt) override;
        void write(WhiArmInterface* HwIf, double Dt) override;
        void quit() override;

    protected:
        void init();
        bool parseConfig(const std::string& Config) override;
        void initializing();
        // TCP protocol related
        std::string packData(const std::string& Command) const;
        template<typename T> struct IsVector : public std::false_type {};
        template<typename T, typename A> struct IsVector<std::vector<T, A>> : public std::true_type {};
        template<typename T> std::string packData(const std::string& Command, const std::vector<T>& Params) const
        {
            if (auto search = CMD_MAP_.find(Command); search != CMD_MAP_.end())
            {
                std::string params("(");
                for (const auto& it : Params)
                {
                    if constexpr (IsVector<T>::value)
                    {
                        if (it.size() > 1)
                        {
                            params += "{";
                        }
                        for (const auto& subIt : it)
                        {
                            if constexpr (std::is_same<T, std::string>::value)
                            {
                                params += subIt + ",";
                            }
                            else
                            {
                                params += std::to_string(subIt) + ",";
                            }
                        }
                        params.pop_back();
                        if (it.size() > 1)
                        {
                            params += "},";
                        }
                        else
                        {
                            params += ",";
                        }
                    }
                    else
                    {
                        if constexpr (std::is_same<T, std::string>::value)
                        {
                            params += it + ",";
                        }
                        else
                        {
                            params += std::to_string(it) + ",";
                        }
                    }
                }
                if (params.length() > 1)
                {
                    params.pop_back();
                }
                params += ")";

                auto index = std::to_string(std::distance(CMD_MAP_.begin(), search));

                return std::string("/f/bIII" + index + "III" + std::to_string(search->second) + "III" +
                    std::to_string(search->first.length() + params.length()) + "III" + search->first + params + "III/b/f");
            }
            else
            {
                return std::string();
            }
        };
        bool parseFeedback(const std::string& Feedback, int& Index, int& ID, std::vector<std::string>& Data) const;
        bool tcp_init();
        bool tcp_close();
        bool tcp_state();
        bool tcp_read();
        bool tcp_servoPositions(const std::vector<double>& Positions, double Duration);
        bool tcp_setIo(int Addr, int Level);
        bool tcp_isProtective();
        bool tcp_protectiveRecover();
        // API related
        bool api_init(const std::string& Addr);
        void api_close();
        bool api_read();
        bool api_servoPositions(const std::vector<double>& Positions, double Duration);
        bool api_setIo(int Addr, int Level);
        bool api_isProtective();
        bool api_protectiveRecover();

    protected:
        enum HomingState { STA_TO_HOME = 0, STA_HOMING, STA_HOMED };

    protected:
        std::shared_ptr<HwConfig> hw_config_{ nullptr };
        std::string controller_type_{ "position" };
        // std::unique_ptr<whi_fair::FRRobot> api_instance_{ nullptr };
        bool is_protective_{ false };
        const std::map<std::string, int> CMD_MAP_{
        {
            {"GetSoftwareVersion", 905},
            {"RobotEnable", 632},
            {"SetAnticollision", 305},
            {"Mode", 303},
            {"SetSpeed", 983},
            {"SetLoadWeight", 306},
            {"SetLoadCoord", 307},
            {"GetActualJointPosRadian", 377},
            {"ServoJ", 376},
            {"SetDO", 204}
        }};
    };
} // namespace whi_arm_interface
