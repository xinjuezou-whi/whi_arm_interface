/******************************************************************
utilities of whi_arm_interface

Features:
- service calling
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/whi_arm_interface_util.h"

namespace whi_arm_hardware_interface
{
    Util::Util(std::shared_ptr<rclcpp::Node>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        srv_io_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvIo>("arm_io",
            std::bind(&Util::onServiceIo, this, std::placeholders::_1, std::placeholders::_2));
        srv_ready_ = node_handle_->create_service<std_srvs::srv::Trigger>("arm_ready",
            std::bind(&Util::onServiceReady, this, std::placeholders::_1, std::placeholders::_2));
    }

    bool Util::onServiceIo(const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Response> Response)
    {
        return Response->result;
    }

    bool Util::onServiceReady(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> Response)
    {
        return Response->success;
    }
} // namespace whi_arm_hardware_interface
