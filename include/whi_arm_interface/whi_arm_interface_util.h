/******************************************************************
utilities of whi_arm_interface

Features:
- service calling
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-06-29: Initial version
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "whi_interfaces/srv/whi_srv_io.hpp"

namespace whi_arm_hardware_interface
{
    class Util
    {
    public:
        Util();
        Util(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~Util() = default;

    protected:
        bool onServiceIo(const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request,
	        std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Response> Response);
        bool onServiceReady(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
	        std::shared_ptr<std_srvs::srv::Trigger::Response> Response);

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        rclcpp::Service<whi_interfaces::srv::WhiSrvIo>::SharedPtr srv_io_{ nullptr };
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ready_{ nullptr };
    };
} // namespace whi_arm_hardware_interface
