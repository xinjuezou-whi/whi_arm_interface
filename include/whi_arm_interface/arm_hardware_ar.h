/******************************************************************
arm hardware interface of ar series under ROS 2
it is a hardware resouces layer for ros_controller

Features:
- currently for ar2 hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-06-16: Initial version
2026-06-25: Migrate to ROS 2
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "arm_hardware_base.h"

#include <serial/serial.h>

namespace whi_arm_interface
{
    // forward declaration
    class HwConfig;

    class ArHardwareInterface : public ArmHardware
    {
    public:
        ArHardwareInterface(const std::string& Config, rclcpp::Node::SharedPtr Node, const std::vector<std::string>& JointNames);
        virtual ~ArHardwareInterface() = default;

    public:
        std::string getSwEstopTopic() const override;
        bool setIo(int Addr, int Level) override;
        void read(WhiArmInterface* HwIf, double Dt) override;
        void write(WhiArmInterface* HwIf, double Dt) override;
        void quit() override;

    protected:
        void init(const std::vector<std::string>& JointNames);
        bool parseConfig(const std::string& Config) override;

    protected:
        void callbackResponse(const std::string& State);

    protected:
        enum HomingState { STA_TO_HOME = 0, STA_HOMING, STA_HOMED };

    protected:
        std::shared_ptr<HwConfig> hw_config_{ nullptr };
        const std::string name_{ "mega2560" };
        std::vector<char> axes_prefix_;
        int homing_state_{ STA_HOMED };
    };
} // namespace whi_arm_interface
