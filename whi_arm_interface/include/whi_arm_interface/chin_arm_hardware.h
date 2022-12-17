/******************************************************************
arm hardware interface of chin under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- chin hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
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

    protected:
        enum HomingState { STA_TO_HOME = 0, STA_HOMING, STA_HOMED };

    protected:
        const std::string name_{ "mega2560" };
        int homing_state_{ STA_HOMED };
        int speed_rate_{ 25 };
        int acc_duration_{ 15 };
        int acc_rate_{ 10 };
        int dec_duration_{ 20 };
        int dec_rate_{ 5 };
    };
}
