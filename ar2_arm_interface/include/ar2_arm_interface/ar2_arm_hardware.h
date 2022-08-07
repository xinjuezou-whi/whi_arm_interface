/******************************************************************
arm hardware interface of ar2 under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- ar2 hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-06-16: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_arm_hardware_base.h"

#include <serial/serial.h>

namespace whi_arm_hardware_interface
{
    class Ar2HardwareInterface : public ArmHardware
    {
    public:
        Ar2HardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~Ar2HardwareInterface() = default;

    protected:
        void init();
        void update(const ros::TimerEvent& Event);
        void read();
        void write(ros::Duration ElapsedTime);

    protected:
        void callbackResponse(const std::string& State);

    protected:
        enum HomingState { STA_TO_HOME = 0, STA_HOMING, STA_HOMED };

    protected:
        const std::string name_{ "mega2560" };
        std::vector<char> axes_prefix_;
        std::vector<double> steps_per_deg_;
        std::vector<int> forward_dir_;
        std::vector<double> home_offsets_;
        std::vector<double> home_kinematics_;
        int homing_state_{ STA_HOMED };
        int speed_rate_{ 25 };
        int acc_duration_{ 15 };
        int acc_rate_{ 10 };
        int dec_duration_{ 20 };
        int dec_rate_{ 5 };
        bool mode_close_loop_{ true };
    };
}
