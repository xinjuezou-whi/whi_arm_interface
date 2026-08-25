/******************************************************************
node to handle arm hardwares
it is a hardware resouces layer for ros_controller

Features:
- hardware resouces setup logic
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-06-13: Initial version
2026-06-24: Migrate to ROS 2
2026-08-12: add OpenArm(Damiao MIT-mode) series support;
            fix on_deactivate() missing hardware_->quit() call;
            fix util node / service / speed_scaling interface name
            collision between left and right hardware instances
2026-08-21: actually wire on_deactivate() to hardware_->quit() (previous
            changelog entry claimed this fix but the call was still a
            bare "// TODO" -- deactivate produced no real de-energize
            command, so the arm stayed enabled until process exit, and
            if the process was SIGTERM-killed before its destructors
            ran, it never got de-energized at all)
2026-08-24: FIX: util_node_ was registered into the shared controller_manager
            executor in on_init() (executor->add_node(util_node_)) but was
            never removed -- see whi_arm_interface.h changelog for the full
            reasoning. Added detachUtilNode(), wired it into new
            on_cleanup()/on_shutdown() overrides, and into a new explicit
            destructor as a fallback net (same "callback + destructor
            fallback" pattern as hardware_->quit(), see on_deactivate()
            below). on_init() now stores Params.executor into executor_weak_
            instead of only using the local `executor` variable, so the
            later removal has something to call remove_node() on.
2026-xx-xx: xxx
******************************************************************/
#include "whi_arm_interface/whi_arm_interface.h"
#include "whi_arm_interface/arm_hardware_ar.h"
#include "whi_arm_interface/arm_hardware_chin.h"
#include "whi_arm_interface/arm_hardware_jaka.h"
#include "whi_arm_interface/arm_hardware_fair.h"
#include "whi_arm_interface/arm_hardware_openarm.h"

namespace whi_arm_interface
{
    WhiArmInterface::~WhiArmInterface()
    {
        // NEW (2026-08-24): fallback net -- see header changelog. Normally
        // on_cleanup()/on_shutdown() will already have done this and
        // detachUtilNode() is a no-op by the time we get here, but if this
        // object is torn down without either lifecycle callback having run
        // (e.g. abrupt teardown, an intervening state-machine path we don't
        // control), this is the last chance to remove util_node_ from the
        // executor before its memory is freed.
        detachUtilNode();
    }

    void WhiArmInterface::detachUtilNode()
    {
        // idempotent: safe to call from on_cleanup(), on_shutdown(), AND
        // the destructor without double-removing or crashing on a second call.
        if (util_node_)
        {
            if (auto executor = executor_weak_.lock())
            {
                executor->remove_node(util_node_);
            }
            util_node_.reset();
        }
    }

    hardware_interface::CallbackReturn WhiArmInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& Params)
    {
        /// node version and copyright announcement
		std::cout << "\nWHI arm interface VERSION 04.09.3" << std::endl;
		std::cout << "Copyright © 2022-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        auto executor = Params.executor.lock();
        if (!executor)
        {
            RCLCPP_FATAL_STREAM(get_logger(), "\033[1;31m" <<
                "cannot lock executor"
            	<< "\033[0m");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // NEW (2026-08-24): keep a weak reference so detachUtilNode() can
        // call remove_node() later (in on_cleanup()/on_shutdown()/the
        // destructor) without needing a fresh Params.executor at that point.
        executor_weak_ = Params.executor;

        if (hardware_interface::SystemInterface::on_init(Params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_FATAL_STREAM(get_logger(), "\033[1;31m" <<
				"failed to load contol params"
				<< "\033[0m");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters["arm_series"] == "ar")
        {
            std::vector<std::string> names;
            for (const auto& it : info_.joints)
            {
                names.push_back(it.name);
            }
            hardware_ = std::make_unique<ArHardwareInterface>(info_.hardware_parameters["hw_config"], get_node(), names);
        }
        else if (info_.hardware_parameters["arm_series"] == "chin")
        {
			hardware_ = std::make_unique<ChinHardwareInterface>(info_.hardware_parameters["hw_config"], get_node());
        }
        else if (info_.hardware_parameters["arm_series"] == "fr")
        {
			hardware_ = std::make_unique<FairHardwareInterface>(info_.hardware_parameters["hw_config"], get_node());
        }
        else if (info_.hardware_parameters["arm_series"] == "jaka")
        {
			hardware_ = std::make_unique<JakaHardwareInterface>(info_.hardware_parameters["hw_config"], get_node());
        }
        else if (info_.hardware_parameters["arm_series"] == "openarm")
        {
            hardware_ = std::make_unique<ArmHardwareOpenarm>(info_.hardware_parameters["hw_config"], get_node());
        }
        else
        {
            RCLCPP_FATAL_STREAM(get_logger(), "\033[1;31m" <<
                "unsupported series"
                << "\033[0m");
            return hardware_interface::CallbackReturn::ERROR;
        }

        std::string srvIo("arm_io");
        std::string srvReady("arm_ready");
        if (info_.hardware_parameters["arm_series"] == "openarm")
        {
            // NOTE: node/service names are prefixed with info_.name (e.g. "openarm_left_hardware_interface")
            // to avoid collisions when multiple WhiArmInterface instances (left/right arm) run in the same process
            util_node_ = std::make_shared<rclcpp::Node>(info_.name + "_util");
            srvIo.assign(info_.name + "_io");
            srvReady.assign(info_.name + "_ready");
        }
        else
        {
            util_node_ = std::make_shared<rclcpp::Node>("whi_arm_interface_util");
        }
        executor->add_node(util_node_);
        srv_io_ = util_node_->create_service<whi_interfaces::srv::WhiSrvIo>(srvIo,
            std::bind(&WhiArmInterface::onServiceIo, this, std::placeholders::_1, std::placeholders::_2));
        srv_ready_ = util_node_->create_service<std_srvs::srv::Trigger>(srvReady,
            std::bind(&WhiArmInterface::onServiceReady, this, std::placeholders::_1, std::placeholders::_2));
        sub_estop_ = util_node_->create_subscription<std_msgs::msg::Bool>(
            hardware_->getSwEstopTopic(), 10, std::bind(&WhiArmInterface::onMsgEstop, this, std::placeholders::_1));

        hw_start_seconds_ = std::max(0.0, stod(info_.hardware_parameters["hw_start_duration_seconds"]));
        hw_stop_seconds_ = std::max(0.0, stod(info_.hardware_parameters["hw_stop_duration_seconds"]));

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn WhiArmInterface::on_configure(
        const rclcpp_lifecycle::State& /*PreState*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

        for (auto i = 0; i < hw_start_seconds_; ++i)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_seconds_ - i);
        }
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        // reset values always when configuring hardware
        // states
        std::fill(hardware_->joint_positions_.begin(), hardware_->joint_positions_.end(), 0.0);
        std::fill(hardware_->joint_velocities_.begin(), hardware_->joint_velocities_.end(), 0.0);
        std::fill(hardware_->joint_acceleration_commands_.begin(), hardware_->joint_acceleration_commands_.end(), 0.0);
        std::fill(hardware_->joint_efforts_.begin(), hardware_->joint_efforts_.end(), 0.0);
        // commands
        std::fill(hardware_->joint_position_commands_.begin(), hardware_->joint_position_commands_.end(), 0.0);
        std::fill(hardware_->joint_velocity_commands_.begin(), hardware_->joint_velocity_commands_.end(), 0.0);
        std::fill(hardware_->joint_effort_commands_.begin(), hardware_->joint_effort_commands_.end(), 0.0);

        RCLCPP_INFO(get_logger(), "Hardware interface successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn WhiArmInterface::on_activate(
        const rclcpp_lifecycle::State& /*PreState*/)
    {
        RCLCPP_INFO(get_logger(), "Activating hardware interface...please wait...");

        for (auto i = 0; i < hw_start_seconds_; ++i)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_seconds_ - i);
        }

        // command and state should be equal when starting
        hardware_->joint_position_commands_ = hardware_->joint_positions_;

        RCLCPP_INFO_STREAM(get_logger(), "\033[1;32m" <<
            "Hardware interface successfully started!"
            << "\033[0m");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn WhiArmInterface::on_deactivate(
        const rclcpp_lifecycle::State& /*PreState*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating hardware interface ...please wait...");

        // NOTE (2026-08-21 fix): this used to be a bare "// TODO" -- deactivate
        // never actually told the hardware layer to de-energize the joints.
        // The only place that really sent the "disable" CAN command was
        // DriverDamiao::close(), which was only reachable via
        // ArmHardwareOpenarm::quit(), which in turn was only ever called from
        // ~ArmHardwareOpenarm(). That means the arm only got de-energized if
        // the process lived long enough for its destructors to run to
        // completion -- if it was SIGTERM-killed first (e.g. after the 5s
        // SIGINT grace period expired), the enable command was never
        // reversed and the arm stayed powered/enabled.
        //
        // hardware_->quit() / DriverDamiao::close() are now idempotent (see
        // driver_damiao.cpp), so it's safe for this to run here AND again
        // later from the destructor as a fallback.
        if (hardware_)
        {
            hardware_->quit();
        }

        for (auto i = 0; i < hw_stop_seconds_; ++i)
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_seconds_ - i);
        }

        RCLCPP_INFO_STREAM(get_logger(), "\033[1;32m" <<
            "Hardware interface successfully stopped!"
            << "\033[0m");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn WhiArmInterface::on_cleanup(
        const rclcpp_lifecycle::State& /*PreState*/)
    {
        // NEW (2026-08-24): see header changelog. Remove util_node_ from the
        // executor here, while executor_weak_ is still very likely to lock
        // successfully (controller_manager is still alive and running this
        // lifecycle transition), rather than leaving it to be discovered
        // only when this object's destructor eventually runs.
        detachUtilNode();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn WhiArmInterface::on_shutdown(
        const rclcpp_lifecycle::State& /*PreState*/)
    {
        // NEW (2026-08-24): some lifecycle paths go straight to shutdown
        // without passing through cleanup first -- detachUtilNode() here
        // covers that path too. Idempotent, so no harm if on_cleanup()
        // already ran.
        detachUtilNode();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> WhiArmInterface::export_state_interfaces()
    {
        hardware_->joint_positions_.resize(info_.joints.size());
        hardware_->joint_velocities_.resize(info_.joints.size());
        hardware_->joint_acceleration_commands_.resize(info_.joints.size());
        hardware_->joint_efforts_.resize(info_.joints.size());

        std::vector<hardware_interface::StateInterface> stateInterfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            stateInterfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hardware_->joint_positions_[i]));

            stateInterfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hardware_->joint_velocities_[i]));

            stateInterfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hardware_->joint_acceleration_commands_[i]));

            stateInterfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hardware_->joint_efforts_[i]));
        }

        stateInterfaces.emplace_back(hardware_interface::StateInterface(
            "speed_scaling", "speed_scaling_factor", &hardware_->speed_scaling_combined_));

        return stateInterfaces;
    }

    std::vector<hardware_interface::CommandInterface> WhiArmInterface::export_command_interfaces()
    {
        hardware_->joint_position_commands_.resize(info_.joints.size());
        hardware_->joint_velocity_commands_.resize(info_.joints.size());
        hardware_->joint_effort_commands_.resize(info_.joints.size());

        auto has_cmd_interface = [](const hardware_interface::ComponentInfo& Joint, const std::string& InterfaceName)
        {
            auto it = find_if(Joint.command_interfaces.begin(), Joint.command_interfaces.end(),
                [&InterfaceName](const hardware_interface::InterfaceInfo& obj)
                {
                    return obj.name == InterfaceName;
                });
            return it != Joint.command_interfaces.end();
        };

        std::vector<hardware_interface::CommandInterface> commandInterfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            commandInterfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hardware_->joint_position_commands_[i]));

            commandInterfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hardware_->joint_velocity_commands_[i]));

            if (has_cmd_interface(info_.joints[i], hardware_interface::HW_IF_EFFORT))
            {
                commandInterfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hardware_->joint_effort_commands_[i]));
            }
        }

        return commandInterfaces;
    }

    hardware_interface::return_type WhiArmInterface::read(const rclcpp::Time& /*Time*/,
        const rclcpp::Duration& Period)
    {
        hardware_->read(this, Period.seconds());

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type WhiArmInterface::write(const rclcpp::Time& /*Time*/,
        const rclcpp::Duration& Period)
    {
        if (!sw_estopped_.load())
        {
            hardware_->write(this, Period.seconds());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "software ESTOP detected! ...please release first and re-try...");
        }

        return hardware_interface::return_type::OK;
    }

    bool WhiArmInterface::onServiceIo(const std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvIo::Response> Response)
    {
        if (Request->io.operation == whi_interfaces::msg::WhiIo::OPER_READ)
        {
            Response->result = false;
        }
        else
        {
            Response->result = hardware_->setIo(Request->io.addr, Request->io.level);
        }
        
        return Response->result;
    }

    bool WhiArmInterface::onServiceReady(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> Response)
    {
        Response->success = hardware_->isStandby();
        return Response->success;
    }

    void WhiArmInterface::onMsgEstop(const std_msgs::msg::Bool::SharedPtr Msg)
    {
        sw_estopped_.store(Msg->data);
    }
}  // namespace whi_arm_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whi_arm_interface::WhiArmInterface, hardware_interface::SystemInterface)
