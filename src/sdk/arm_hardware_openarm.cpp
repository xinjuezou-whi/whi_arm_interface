/******************************************************************
motion hardware interface of robotic arm (Damiao MIT-mode joints) under ROS 2

Features:
- OpenArm hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com
Apache License Version 2.0, check LICENSE for more information.

******************************************************************/
#include "whi_arm_interface/arm_hardware_openarm.h"
#include "whi_arm_interface/driver_damiao.h"

#include <yaml-cpp/yaml.h>

namespace whi_arm_interface
{
    ArmHardwareOpenarm::ArmHardwareOpenarm(const std::string& Config, rclcpp::Node::SharedPtr Node)
        : ArmHardware(Config, Node)
    {
        if (parseConfig(Config))
        {
            hw_config_->printOut();
            init();
        }
    }

    ArmHardwareOpenarm::~ArmHardwareOpenarm()
    {
        quit();
    }

    std::string ArmHardwareOpenarm::getSwEstopTopic() const
    {
        return hw_config_ ? hw_config_->sw_estop_topic_ : std::string("estop");
    }

    bool ArmHardwareOpenarm::setIo(int /*Addr*/, int /*Level*/)
    {
        // 达妙关节暂无独立IO能力
        return false;
    }

    bool ArmHardwareOpenarm::parseConfig(const std::string& Config)
    {
        try
        {
            hw_config_ = std::make_shared<HwConfig>();
            YAML::Node node = YAML::LoadFile(Config);

            const auto& root = node["whi_arm_interface"];
            if (root)
            {
                const auto& estopTopic = root["sw_estop_topic"];
                if (estopTopic)
                {
                    hw_config_->sw_estop_topic_ = estopTopic.as<std::string>();
                }

                const auto& canbus = root["canbus"];
                if (canbus)
                {
                    for (const auto& it : canbus)
                    {
                        HwConfig::Motor motor;
                        motor.protocol_config_ = it.second["protocol_config"].as<std::string>();
                        motor.bus_addr_ = it.second["bus_addr"].as<std::string>();
                        motor.device_addr_ = it.second["device_addr"].as<int>();
                        motor.recv_device_addr_ = it.second["recv_device_addr"].as<int>();
                        motor.mit_kp_ = it.second["mit_kp"].as<double>();
                        motor.mit_kd_ = it.second["mit_kd"].as<double>();

                        const auto& driveType = it.second["drive_type"];
                        if (driveType)
                        {
                            const auto driveTypeMap = driveType.as<std::map<std::string, int>>();
                            if (!driveTypeMap.empty())
                            {
                                motor.drive_type_.first = driveTypeMap.begin()->first;
                                motor.drive_type_.second = driveTypeMap.begin()->second;
                            }
                        }

                        motor.forward_dir_ = it.second["forward_dir"].as<int>();
                        motor.resolution_ = it.second["resolution"].as<int>();
                        motor.multiple_ = it.second["multiple"].as<bool>();

                        joint_names_.push_back(it.first.as<std::string>());
                        hw_config_->motors_map_.emplace(joint_names_.back(), std::move(motor));
                    }
                }

                const auto& debug = root["debug"];
                if (debug)
                {
                    const auto& printConfig = debug["print_config"];
                    if (printConfig)
                    {
                        hw_config_->debug_print_config_ = printConfig.as<bool>();
                    }
                }

                return true;
            }
            else
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                    "failed to find whi_arm_interface properties in " << Config
                    << "\033[0m");
                hw_config_.reset();
                return false;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "failed to load openarm hardware config " << Config << " with error: " << e.what() << std::endl;
            hw_config_.reset();
            return false;
        }
    }

    void ArmHardwareOpenarm::init()
    {
        for (const auto& [name, motor] : hw_config_->motors_map_)
        {
            auto driver = std::make_unique<DriverDamiao>(name, motor.bus_addr_,
                uint16_t(motor.device_addr_), uint16_t(motor.recv_device_addr_),
                motor.drive_type_.second, /*DriveMode*/0, /*ReductionRatio*/1.0);
            driver->parseProtocolConfig(motor.protocol_config_);
            driver->setGains(motor.mit_kp_, motor.mit_kd_);

            drivers_map_.emplace(name, std::move(driver));
        }

        initialized_ = !drivers_map_.empty();
    }

    void ArmHardwareOpenarm::read(WhiArmInterface* /*HwIf*/, double /*Dt*/)
    {
        for (std::size_t i = 0; i < joint_names_.size(); ++i)
        {
            joint_positions_[i] = drivers_map_.at(joint_names_[i])->readAngle();
            // 速度/力矩若需要，通过 dynamic_cast<DriverDamiao*> 拿 readVelocity()/readTorque()
            auto damiao = dynamic_cast<DriverDamiao*>(drivers_map_.at(joint_names_[i]).get());
            if (damiao)
            {
                joint_velocities_[i] = damiao->readVelocity();
                joint_efforts_[i] = damiao->readTorque();
            }
        }
    }

    void ArmHardwareOpenarm::write(WhiArmInterface* /*HwIf*/, double /*Dt*/)
    {
        for (std::size_t i = 0; i < joint_names_.size(); ++i)
        {
            drivers_map_.at(joint_names_[i])->actuate(joint_position_commands_[i]);
        }
    }

    void ArmHardwareOpenarm::quit()
    {
        for (auto& it : drivers_map_)
        {
            it.second->close();
        }
    }
} // namespace whi_arm_interface
