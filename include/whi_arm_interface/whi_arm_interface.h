/******************************************************************
arm hardware interface under ROS 2
it is a hardware resouces layer for ros_control

Features:
- abstract arm hardware interfaces
- xxx

Dependency:
- sudo apt install apt ros-<distro>-joint-trajectory-controller

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-06-16: Initial version
2026-06-24: Migrate to ROS 2
2026-08-24: FIX: on_init() registers util_node_ into the shared
            controller_manager executor via executor->add_node(util_node_)
            but nothing ever called executor->remove_node(util_node_).
            The executor is typically spun on its own thread by
            controller_manager, independent of when/how this hardware
            component's destructor runs. Without removing util_node_
            first, destroying it (as a member of this object, during
            teardown) can race with the executor thread still iterating
            its registered nodes / touching util_node_'s internals --
            a cross-thread use-after-free. This is a plausible root
            cause for a heap corruption that only aborts much later, on
            an unrelated allocation (observed: freeing the second/left
            hardware instance's HwConfig shared_ptr at shutdown, right
            after the first/right instance tore down cleanly -- exactly
            the kind of "silent corruption now, abort on some later
            unrelated free()" signature this class of bug produces).
            Added on_cleanup()/on_shutdown() overrides that remove
            util_node_ from the executor while the executor is still
            known-valid, plus a destructor as a fallback net in case
            neither lifecycle callback runs before this object is torn
            down (mirrors the same "callback + destructor fallback"
            pattern already used for hardware_->quit() -- see
            on_deactivate() below and DriverDamiao::close()).
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "arm_hardware_base.h"
#include "whi_interfaces/srv/whi_srv_io.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

namespace whi_arm_interface
{
	/// brief Hardware interface for a robot
	class WhiArmInterface : public hardware_interface::SystemInterface
	{
	public:
        RCLCPP_SHARED_PTR_DEFINITIONS(WhiArmInterface)

        // NEW (2026-08-24): explicit destructor -- fallback net that removes
        // util_node_ from the executor if neither on_cleanup() nor
        // on_shutdown() ran before this object is torn down. See changelog
        // above for the full reasoning.
        ~WhiArmInterface() override;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& Params) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& PreState) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& PreState) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& PreState) override;
        // NEW (2026-08-24): see changelog above.
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& PreState) override;
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& PreState) override;

		std::vector<hardware_interface::StateInterface> export_state_interfaces() final;
		std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

        hardware_interface::return_type read(const rclcpp::Time& Time,
            const rclcpp::Duration& Period) override;
        hardware_interface::return_type write(const rclcpp::Time& Time,
            const rclcpp::Duration& Period) override;

    protected:
        bool onServiceIo(const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request,
	        std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Response> Response);
        bool onServiceReady(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
	        std::shared_ptr<std_srvs::srv::Trigger::Response> Response);
        void onMsgEstop(const std_msgs::msg::Bool::SharedPtr Msg);

        // NEW (2026-08-24): shared implementation for on_cleanup()/
        // on_shutdown()/~WhiArmInterface() -- all three need to do the same
        // "remove util_node_ from the executor, exactly once" step. Safe to
        // call more than once (idempotent: no-ops once util_node_ is reset).
        void detachUtilNode();

    private:
        // Parameters for the simulation
        double hw_start_seconds_{ 0.2 };
        double hw_stop_seconds_{ 1.0 };

	protected:
		std::unique_ptr<ArmHardware> hardware_{ nullptr };
        rclcpp::Node::SharedPtr util_node_{ nullptr };
        // NEW (2026-08-24): weak reference to the executor util_node_ was
        // registered into (see on_init()), kept so detachUtilNode() can
        // call remove_node() later without needing a fresh Params.executor.
        // weak_ptr rather than shared_ptr deliberately -- this object must
        // never be the thing keeping controller_manager's executor alive.
        rclcpp::Executor::WeakPtr executor_weak_{};
        rclcpp::Service<whi_interfaces::srv::WhiSrvIo>::SharedPtr srv_io_{ nullptr };
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_ready_{ nullptr };
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_{ nullptr };
        std::atomic_bool sw_estopped_{ false };
	};
} // namespace whi_arm_interface
