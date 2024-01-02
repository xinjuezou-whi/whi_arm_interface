/******************************************************************
arm hardware interface of chin under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- chin hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-12-16: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_arm_hardware_base.h"

namespace whi_arm_hardware_interface
{
    class ChinHardwareInterface : public ArmHardware
    {
    public:
        ChinHardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~ChinHardwareInterface() = default;

    protected:
        void init();
        void update(const ros::TimerEvent& Event);
        void read();
        void write(ros::Duration ElapsedTime);
        std::string composeCommand(const std::string& Positions) const;
        std::string composeCommand(const std::string& Positions, const std::string& Velocities,
            const std::string& Accelerations) const;

    protected:
        enum HomingState { STA_TO_HOME = 0, STA_HOMING, STA_HOMED };

    protected:
        const std::string name_{ "socket" };
        std::string controller_type_{ "position" };
        int homing_state_{ STA_HOMED };
        double speed_rate_{ 0.5 };
        std::vector<double> forward_dirs_;
        std::vector<double> angulars_;
        std::vector<double> accelerations_;
    };
}
