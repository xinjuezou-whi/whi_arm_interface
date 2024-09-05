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
    public:
        JakaHardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~JakaHardwareInterface();

    protected:
        void init();
        void update(const ros::TimerEvent& Event);
        void read();
        void write(ros::Duration ElapsedTime);
        // jakaAPI related
        bool jaka_api_init(const std::string& Addr);
        void jaka_api_close();
        std::vector<double> jaka_api_readPositions() const;
        void jaka_api_servoPositions(const std::vector<double>& Positions, double Duration);

    protected:
        enum HomingState { STA_TO_HOME = 0, STA_HOMING, STA_HOMED };

    protected:
        const std::string name_{ "socket" };
        std::string controller_type_{ "position" };
        std::unique_ptr<JAKAZuRobot> jaka_api_instance_{ nullptr };
    };
}
