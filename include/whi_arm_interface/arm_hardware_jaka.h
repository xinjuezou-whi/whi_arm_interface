/******************************************************************
arm hardware interface of JAKA under ROS 2
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
2026-06-25：Migrate to ROS 2
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "arm_hardware_base.h"
#include "whi_interfaces/srv/whi_srv_io.hpp"
#include "jakaAPI/JAKAZuRobot.h"

#include <std_srvs/srv/trigger.hpp>

namespace whi_arm_hardware_interface
{
    // forward declaration
    class HwConfig;

    class JakaHardwareInterface : public ArmHardware
    {
    protected:
        enum ParamKey { JOINT_POS = 0, PROTECTIVE_STOP, ENABLE, POWER, IN_SERVO, PARAM_KEY_SUM };
        static constexpr const char* paramKey[PARAM_KEY_SUM] = { "joint_pos", "protective_stop", "enable", "power", "in_servomove" };

    public:
        JakaHardwareInterface(const std::string& Config, rclcpp::Node::SharedPtr Node);
        virtual ~JakaHardwareInterface();

    public:
        bool setIo(int Addr, int Level) override;
        void read(WhiArmInterface* HwIf, double Dt) override;
        void write(WhiArmInterface* HwIf, double Dt) override;
        void quit() override;

    protected:
        void init();
        bool parseConfig(const std::string& Config) override;
        void initializing();
        // TCP protocol related
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
        std::unique_ptr<JAKAZuRobot> api_instance_{ nullptr };
        std::vector<double> payload_to_tcp_;
        bool is_protective_{ false };
        double lpf_{ 0.5 };
    };
}
