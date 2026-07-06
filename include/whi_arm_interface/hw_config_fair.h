/******************************************************************
hardware configure structure for Fairino series manipulator

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
        static void printOut(const HwConfig& Config)
        {
            std::cout << "******************** HwConfig" << std::endl;
            std::cout << "shutdown_patience_: " << Config.shutdown_patience_ << std::endl;
            std::cout << "velocity_scale_: " << Config.velocity_scale_ << std::endl;
            std::cout << "payload_weight_: " << Config.payload_weight_ << std::endl;
            std::cout << "payload_to_tcp_: ";
            for (const auto& it : Config.payload_to_tcp_)
            {
                std::cout << it << ", ";
            }
            std::cout << std::endl;
            std::cout << "hardware_: " << Config.hardware_ << std::endl;
            std::cout << "startup_duration_: " << Config.startup_duration_ << std::endl;
            std::cout << "socket_addr_: " << Config.socket_addr_ << std::endl;
            std::cout << "socket_port_: " << Config.socket_port_ << std::endl;
            std::cout << "api_addr_: " << Config.api_addr_ << std::endl;
            std::cout << "debug_print_tcp_feedback_: " << (Config.debug_print_tcp_feedback_ ? "true" : "false") << std::endl;
            std::cout << "******************** end of HwConfig" << std::endl;
        }

    public:
        std::string sw_estop_topic_{ "estop" };
        int shutdown_patience_{ 5000 };
        double velocity_scale_{ 1.0 };
        double payload_weight_{ 0.0 };
        std::vector<double> payload_to_tcp_;
        std::string hardware_{ "socket" };
        double startup_duration_{ 10.0 };
        std::string socket_addr_{ "192.168.4.44" };
        int socket_port_{ 9866 };
        std::string api_addr_{ "192.168.4.44" };
        bool debug_print_tcp_feedback_{ false };
    };
} // namespace whi_arm_interface
