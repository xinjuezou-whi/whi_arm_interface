/******************************************************************
arm hardware interface of chin under ROS 2
it is a hardware resouces layer for ros_controller

Features:
- chin hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-12-16: Initial version
2026-06-25: Migrate to ROS 2
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "arm_hardware_base.h"

namespace whi_arm_hardware_interface
{
    // forward declaration
    class HwConfig;

    class ChinHardwareInterface : public ArmHardware
    {
    public:
        ChinHardwareInterface(const std::string& Config, rclcpp::Node::SharedPtr Node);
        virtual ~ChinHardwareInterface();
    
    public:
        bool setIo(int Addr, int Level) override;
        void read(WhiArmInterface* HwIf, double Dt) override;
        void write(WhiArmInterface* HwIf, double Dt) override;
        void quit() override;

    protected:
        void init();
        bool parseConfig(const std::string& Config) override;
        std::string composeCommand(const std::string& Positions) const;
        std::string composeCommand(const std::string& Positions, const std::string& Velocities,
            const std::string& Accelerations) const;

    protected:
        std::shared_ptr<HwConfig> hw_config_{ nullptr };
        std::string controller_type_{ "position" };
    };
}
