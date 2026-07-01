/******************************************************************
hardware configure structure for AR series manipulator

Features:
- hardware configure parameters
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-06-25: Initial version
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <map>

namespace whi_arm_hardware_interface
{
    /// config
    class HwConfig
    {
    public:
        HwConfig() = default;
        ~HwConfig() = default;

    public:
        static void printOut(const HwConfig& Config)
        {
            std::cout << "******************** HwConfig" << std::endl;
            std::cout << "shutdown_patience_: " << Config.shutdown_patience_ << std::endl;
            std::cout << "steps_per_degree_: ";
            for (const auto& it : Config.steps_per_degree_)
            {
                std::cout << it << ", ";
            }
            std::cout << std::endl;
            std::cout << "forward_dirs_: ";
            for (const auto& it : Config.forward_dirs_)
            {
                std::cout << it << ", ";
            }
            std::cout << std::endl;
            std::cout << "limits_dirs_: " << std::endl;
            for (const auto& it : Config.limits_dirs_)
            {
                std::cout << it << ", ";
            }
            std::cout << std::endl;
            std::cout << "home_offsets_: ";
            for (const auto& it : Config.home_offsets_)
            {
                std::cout << it << ", ";
            }
            std::cout << std::endl;
            std::cout << "home_kinematics_: ";
            for (const auto& it : Config.home_kinematics_)
            {
                std::cout << it << ", ";
            }
            std::cout << std::endl;
            std::cout << "speed_rate_: " << Config.speed_rate_ << std::endl;
            std::cout << "acc_duration_: " << Config.acc_duration_ << std::endl;
            std::cout << "acc_rate_: " << Config.acc_rate_ << std::endl;
            std::cout << "dec_duration_: " << Config.dec_duration_ << std::endl;
            std::cout << "dec_rate_: " << Config.dec_rate_ << std::endl;
            std::cout << "close_mode_: " << (Config.close_mode_ ? "true" : "false") << std::endl;
            std::cout << "home_poweron_: " << (Config.home_poweron_ ? "true" : "false") << std::endl;
            std::cout << "hardware_: " << Config.hardware_ << std::endl;
            std::cout << "serial_port_: " << Config.serial_port_ << std::endl;
            std::cout << "serial_baudrate_: " << Config.serial_baudrate_ << std::endl;
            std::cout << "******************** end of HwConfig" << std::endl;
        }

    public:
        std::string sw_estop_topic_{ "estop" };
        int shutdown_patience_{ 5000 };
        std::vector<double> steps_per_degree_;
        std::vector<int> forward_dirs_;
        std::vector<int> limits_dirs_;
        std::vector<double> home_offsets_;
        std::vector<double> home_kinematics_;
        int speed_rate_{ 25 };
        int acc_duration_{ 15 };
        int acc_rate_{ 10 };
        int dec_duration_{ 20 };
        int dec_rate_{ 5 };
        bool close_mode_{ true };
        bool home_poweron_{ true };
        std::string hardware_{ "serial" };
        std::string serial_port_{ "/dev/ttyACM0" };
        int serial_baudrate_{ 9600 };
    };
} // namespace whi_arm_hardware_interface
