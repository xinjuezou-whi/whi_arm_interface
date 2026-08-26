/******************************************************************
motion hardware interface of robotic arm (Damiao MIT-mode joints) under ROS 2

Features:
- OpenArm hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com
Apache License Version 2.0, check LICENSE for more information.

Changelog:
2026-08-11: Initial version
2026-xx-xx: xx
******************************************************************/
#pragma once
#include "arm_hardware_base.h"
#include "hw_config_openarm.h"

#include <map>
#include <memory>
#include <vector>
#include <string>

namespace whi_arm_interface
{
    class ArmHardwareOpenarm : public ArmHardware
    {
    public:
        ArmHardwareOpenarm() = delete;
        ArmHardwareOpenarm(const std::string& Config, rclcpp::Node::SharedPtr Node);
        virtual ~ArmHardwareOpenarm();

    public:
        std::string getSwEstopTopic() const override;
        bool setIo(int Addr, int Level) override;
        void read(WhiArmInterface* HwIf, double Dt) override;
        void write(WhiArmInterface* HwIf, double Dt) override;
        void quit() override;

    protected:
        bool parseConfig(const std::string& Config) override;
        void init();

    protected:
        // std::shared_ptr<HwConfig> hw_config_{ nullptr };
        HwConfig hw_config_;
        std::vector<std::string> joint_names_; // for specific order of joints
    };
} // namespace whi_arm_interface
