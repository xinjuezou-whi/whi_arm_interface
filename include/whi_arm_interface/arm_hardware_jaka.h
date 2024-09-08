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
#include "whi_arm_hardware_base.h"
#include "jakaAPI/JAKAZuRobot.h"

namespace whi_arm_hardware_interface
{
    class JakaHardwareInterface : public ArmHardware
    {
    protected:
        enum ParamKey { JOINT_POS = 0, PARAM_KEY_SUM };
        static constexpr const char* paramKey[PARAM_KEY_SUM] = { "joint_pos" };

    public:
        JakaHardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~JakaHardwareInterface();

    protected:
        void init();
        void update(const ros::TimerEvent& Event);
        void read();
        void write(ros::Duration ElapsedTime);
        // jaka TCP protocol related
        bool jaka_tcp_init();
        void jaka_tcp_close();
        bool jaka_tcp_read();
        void jaka_tcp_servoPositions(const std::vector<double>& Positions, double Duration);
        // jakaAPI related
        bool jaka_api_init(const std::string& Addr);
        void jaka_api_close();
        bool jaka_api_read();
        void jaka_api_servoPositions(const std::vector<double>& Positions, double Duration);

    protected:
        enum HomingState { STA_TO_HOME = 0, STA_HOMING, STA_HOMED };

    protected:
        std::string name_{ "socket" };
        std::string controller_type_{ "position" };
        std::unique_ptr<JAKAZuRobot> jaka_api_instance_{ nullptr };
        double velocity_scale_{ 1.0 };
        bool initialized_{ false };
    };
}
