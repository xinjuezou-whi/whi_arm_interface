/******************************************************************
arm hardware interface of ar2 under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- ar2 hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/arm_hardware_ar.h"
#include "whi_arm_interface/driver_serial.h"
#include "whi_arm_interface/hw_config_ar.h"

#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
#include <std_msgs/msg/string.hpp>

#include <thread>

namespace whi_arm_hardware_interface
{
    ArHardwareInterface::ArHardwareInterface(const std::string& Config, rclcpp::Node::SharedPtr Node, const std::vector<std::string>& JointNames)
        : ArmHardware(Config, Node)
    {
        parseConfig(Config);
        init(JointNames);
    }

    void ArHardwareInterface::quit()
    {
        // give time to thirdparty dependencies
        std::this_thread::sleep_for(std::chrono::milliseconds(hw_config_->shutdown_patience_));
    }

    void ArHardwareInterface::init(const std::vector<std::string>& JointNames)
    {
        for (std::size_t i = 0; i < JointNames.size(); ++i)
        {
            // A B C D E F...
            axes_prefix_.push_back(char(65 + i));
        }

        homing_state_ = hw_config_->home_poweron_ ? STA_TO_HOME : STA_HOMED;

        // drivers
        if (hw_config_->hardware_ == hardware[SERIAL])
        {
            // currently AR2's arduino accept single combined command,
            // therefore init one serial instance
            try
            {
                auto serialInst = std::make_shared<serial::Serial>(hw_config_->serial_port_, hw_config_->serial_baudrate_,
                    serial::Timeout::simpleTimeout(500));
                drivers_map_.emplace(name_, std::make_unique<DriverSerial>(name_, serialInst));
                ((DriverSerial*)drivers_map_[name_].get())->setMotor(hw_config_->limits_dirs_);
            }
            catch (serial::IOException& e)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                    "failed to open serial " << hw_config_->serial_port_
                    << "\033[0m");
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to init driver of " << hw_config_->hardware_
                << "\033[0m");
        }
    }

    std::string ArHardwareInterface::getSwEstopTopic() const
    {
        return hw_config_ ? hw_config_->sw_estop_topic_ : "na";
    }

    bool ArHardwareInterface::setIo(int Addr, int Level)
    {
        // nothing, so far
        return false;
    }

    void ArHardwareInterface::read(WhiArmInterface* HwIf, double Dt)
    {
        // do nothing, since the position is updated by message callback
    }

    void ArHardwareInterface::write(WhiArmInterface* HwIf, double Dt)
    {
        if (homing_state_ == STA_HOMED)
        {
            std::string cmd("MJ");
            for (std::size_t i = 0; i < joint_position_commands_.size(); ++i)
            {
                double degCmd = angles::to_degrees(joint_position_commands_[i]);
                double degCur = angles::to_degrees(joint_positions_[i]);
                int step = int((degCmd - degCur) * hw_config_->steps_per_degree_[i]) * hw_config_->forward_dirs_[i];
                cmd.append(std::string(1, axes_prefix_[i]) + (step >= 0 ? "1" : "0") + std::to_string(abs(step)));
            }
            cmd.append(std::string("S") + std::to_string(hw_config_->speed_rate_) +
                "G" + std::to_string(hw_config_->acc_duration_) + "H" + std::to_string(hw_config_->acc_rate_) +
                "I" + std::to_string(hw_config_->dec_duration_) + "K" + std::to_string(hw_config_->dec_rate_));

            drivers_map_[name_]->actuate(cmd);
#ifdef DEBUG
            std::cout << "arduino cmd " << cmd << std::endl;
#endif
            if (!hw_config_->close_mode_)
            {
                // update current to command
                for (std::size_t i = 0; i < joint_positions_.size(); ++i)
                {
                    joint_positions_[i] = joint_position_commands_[i];
                }
            }
        }
        else if (homing_state_ == STA_TO_HOME)
        {
            std::string cmd("hm");
            for (std::size_t i = 0; i < joint_position_commands_.size(); ++i)
            {
                int step = int(hw_config_->home_offsets_[i] * hw_config_->steps_per_degree_[i]) * hw_config_->forward_dirs_[i];
                cmd.append(std::string(1, axes_prefix_[i]) + (step >= 0 ? "1" : "0") + std::to_string(abs(step)));
            }
            cmd.append(std::string("S") + std::to_string(int(hw_config_->home_kinematics_[0])) +
                "G" + std::to_string(int(hw_config_->home_kinematics_[1])) + "H" + std::to_string(int(hw_config_->home_kinematics_[2])) +
                "I" + std::to_string(int(hw_config_->home_kinematics_[3])) + "K" + std::to_string(int(hw_config_->home_kinematics_[4])) +
                "l");
            for (const auto& it : hw_config_->limits_dirs_)
            {
                cmd.append(it > 0 ? "1" : "0");
            }

            drivers_map_[name_]->actuate(cmd);
#ifdef DEBUG
            std::cout << "arduino cmd " << cmd << std::endl;
#endif
        }
    }

    bool ArHardwareInterface::parseConfig(const std::string& Config)
    {
        try
        {
            hw_config_ = std::make_shared<HwConfig>();

            // from hardware's yaml
            YAML::Node node = YAML::LoadFile(Config);

            const auto& root = node["whi_arm_interface"];
            if (root)
            {
                hw_config_->sw_estop_topic_ = root["sw_estop_topic"].as<std::string>();
                hw_config_->shutdown_patience_ = root["shutdown_patience"].as<int>();
                hw_config_->steps_per_degree_ = root["steps_per_degree"].as<std::vector<double>>();
                hw_config_->forward_dirs_ = root["forward_dirs"].as<std::vector<int>>();
                hw_config_->limits_dirs_ = root["limits_dirs"].as<std::vector<int>>();
                hw_config_->home_offsets_ = root["home_offsets"].as<std::vector<double>>();
                hw_config_->home_kinematics_ = root["home_kinematics"].as<std::vector<double>>();
                hw_config_->speed_rate_ = root["speed_rate"].as<int>();
                hw_config_->acc_duration_ = root["acc_duration"].as<int>();
                hw_config_->acc_rate_ = root["acc_rate"].as<int>();
                hw_config_->dec_duration_ = root["dec_duration"].as<int>();
                hw_config_->dec_rate_ = root["dec_rate"].as<int>();
                hw_config_->home_poweron_ = root["home_poweron"].as<bool>();
                const auto& controlMode = root["control_mode"].as<std::string>();
                hw_config_->close_mode_ = (controlMode == "close");
                hw_config_->hardware_ = root["hardware"].as<std::string>();

                const auto& serial = root["serial"];
                if (serial)
                {
                    hw_config_->serial_port_ = serial["port"].as<std::string>();
                    hw_config_->serial_baudrate_ = serial["baudrate"].as<int>();
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
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to load hardware config file " << Config << " with error: " << e.what()
                << "\033[0m");
            return false;
        }
    }

    void ArHardwareInterface::callbackResponse(const std::string& State)
    {
        if (State.find("homing") != std::string::npos)
        {
            homing_state_ = STA_HOMING;
            std::cout << "start to homing..." << std::endl;
        }
        else if (State.find("homed") != std::string::npos)
        {
            homing_state_ = STA_HOMED;
            std::cout << "homed successfully" << std::endl;
        }
        else if (State.find("p") != std::string::npos)
        {
            if (hw_config_->close_mode_)
            {
                try
                {
                    std::size_t begin = 0;
                    std::size_t end = 0;
                    for (std::size_t i = 0; i < joint_positions_.size(); ++i)
                    {
                        begin = State.find('p', begin);
                        end = State.find('p', begin + 1);
                        if (end > begin)
                        {
                            joint_positions_[i] = angles::from_degrees(
                                hw_config_->forward_dirs_[i] * std::stoi(State.substr(begin + 1, end - begin - 1)) / hw_config_->steps_per_degree_[i]);
                            begin = end;
                        }
                    }
                }
                catch (const std::exception& e)
                {
                    std::cout << "tranferred data exception" << std::endl;
                }
            }
        }
    }
}
