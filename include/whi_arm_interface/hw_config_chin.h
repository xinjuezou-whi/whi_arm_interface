/******************************************************************
hardware configure structure for Chin series manipulator

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

namespace whi_arm_interface
{
    /// config
    class HwConfig
    {
    public:
        HwConfig() = default;
        ~HwConfig() = default;

    public:
        void printOut()
        {
            if (debug_print_config_)
            {
                std::cout << "******************** HwConfig" << std::endl;
                std::cout << "hardware_: " << hardware_ << std::endl;
                std::cout << "forward_dirs_: ";
                for (const auto& it : forward_dirs_)
                {
                    std::cout << it << ", ";
                }
                std::cout << std::endl;
                std::cout << "speed_rate_: " << speed_rate_ << std::endl;
                std::cout << "angular_velocities_: " << std::endl;
                for (const auto& it : angular_velocities_)
                {
                    std::cout << it << ", ";
                }
                std::cout << std::endl;
                std::cout << "angular_accelerations_: ";
                for (const auto& it : angular_accelerations_)
                {
                    std::cout << it << ", ";
                }
                std::cout << std::endl;
                std::cout << "socket_addr_: " << socket_addr_ << std::endl;
                std::cout << "socket_port_: " << socket_port_ << std::endl;
                std::cout << "******************** end of HwConfig" << std::endl;
            }
        }

    public:
        std::string sw_estop_topic_{ "estop" };
        std::string hardware_{ "socket" };
        std::vector<int> forward_dirs_;
        double speed_rate_{ 1.0 };
        std::vector<double> angular_velocities_;
        std::vector<double> angular_accelerations_;
        std::string socket_addr_{ "192.168.4.44" };
        int socket_port_{ 9866 };
        bool debug_print_config_{ false };
    };
} // namespace whi_arm_interface
