/******************************************************************
arm hardware interface of chin under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- chin hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/arm_hardware_chin.h"
#include "whi_arm_interface/driver_socket.h"
#include "whi_arm_interface/hw_config_chin.h"

#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>

#include <thread>

namespace whi_arm_hardware_interface
{
    ChinHardwareInterface::ChinHardwareInterface(const std::string& Config, rclcpp::Node::SharedPtr Node)
        : ArmHardware(Config, Node)
    {
        parseConfig(Config);
        init();
    }

    ChinHardwareInterface::~ChinHardwareInterface()
    {
        for (int i = 0; i < 3; ++i)
		{
			((DriverSocket*)drivers_map_[hw_config_->hardware_].get())->sendCommand("SHUT");
			usleep(200000);
		}
    }

    void ChinHardwareInterface::quit()
    {
        // give time to thirdparty dependencies
        std::this_thread::sleep_for(std::chrono::milliseconds(hw_config_->shutdown_patience_));
    }

    void ChinHardwareInterface::init()
    {
        // drivers
        if (hw_config_->hardware_ == hardware[SOCKET])
        {
            drivers_map_.emplace(hw_config_->hardware_, std::make_unique<DriverSocket>(hw_config_->hardware_,
                hw_config_->socket_addr_, hw_config_->socket_port_));
            ((DriverSocket*)drivers_map_[hw_config_->hardware_].get())->setMotor();
        }
        else
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to init driver of " << hw_config_->hardware_
                << "\033[0m");
        }
    }

    std::string ChinHardwareInterface::getSwEstopTopic() const
    {
        return hw_config_ ? hw_config_->sw_estop_topic_ : "na";
    }

    bool ChinHardwareInterface::setIo(int Addr, int Level)
    {
        // nothing, so far
        return false;
    }

    bool ChinHardwareInterface::parseConfig(const std::string& Config)
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
                hw_config_->hardware_ = root["hardware"].as<std::string>();
                hw_config_->forward_dirs_ = root["forward_dirs"].as<std::vector<int>>();
                hw_config_->speed_rate_ = root["speed_rate"].as<int>();
                hw_config_->angular_velocities_ = root["angular_velocities"].as<std::vector<double>>();
                hw_config_->angular_accelerations_ = root["angular_accelerations"].as<std::vector<double>>();

                const auto& socket = root["socket"];
                if (socket)
                {
                    hw_config_->socket_addr_ = socket["addr"].as<std::string>();
                    hw_config_->socket_port_ = socket["port"].as<int>();
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

    void ChinHardwareInterface::read(WhiArmInterface* HwIf, double Dt)
    {
        static bool init = true;

        if (((DriverSocket*)drivers_map_[hw_config_->hardware_].get())->isServoOn(3000))
        {
            std::vector<std::string> params { "ANGLES", "DANGLES" };
            ((DriverSocket*)drivers_map_[hw_config_->hardware_].get())->request(params);
            std::vector<double> angles = ((DriverSocket*)drivers_map_[hw_config_->hardware_].get())->readParam(params[0]);
            for (std::size_t i = 0; i < 
                std::min(std::min(joint_positions_.size(), angles.size()), hw_config_->forward_dirs_.size()); ++i)
            {
                joint_positions_[i] = angles::from_degrees(hw_config_->forward_dirs_[i] * angles[i]);
            }
            std::vector<double> velocities = ((DriverSocket*)drivers_map_[hw_config_->hardware_].get())->readParam(params[1]);
            for (std::size_t i = 0; i < 
                std::min(std::min(joint_velocities_.size(), velocities.size()), hw_config_->forward_dirs_.size()); ++i)
            {
                joint_velocities_[i] = angles::from_degrees(hw_config_->forward_dirs_[i] * velocities[i]);
            }
#ifdef DEBUG
            std::cout << "velocity read" << std::endl;
            for (const auto& it : joint_velocities_)
            {
                std::cout << it << ",";
            }
            std::cout << std::endl;
#endif
            // there's no acceleration data available in joint state handle
        }
        if (init)
        {
            joint_position_commands_ = joint_positions_;
            joint_velocity_commands_ = joint_velocities_;
            init = false;
        }        
    }

#ifdef DEBUG
    // following commands refer to demo.py of Chin
    std::vector<std::string> commands_demo;
    void initDebugCommandList()
    {
        commands.push_back("MOVEJ,DOF,-11.709996,-1.34654,-119.936058,-28.582537,-89.999959,-11.709957,DOF,99,99,99,99,99,99,DOF,198,198,198,198,198,198,0");
        commands.push_back("MOVEL,TL,505.091468,-99.512703,159.2,RPY,-180,0,90,1111,1111,0");
        commands.push_back("MOVELN,TL,508.087747,-111.943565,159.2,1111,1111,0");
        commands.push_back("MOVEAN,TL,514.284471,-114.968887,159.2,TL,518.138528,-117.94697,159.2,1111,1111,0");
        commands.push_back("MOVEAN,TL,528.107276,-121.416063,159.2,TL,526.24835,-131.271895,159.2,1111,1111,0");
        commands.push_back("MOVELN,TL,527.488383,-142.14683,159.2,1111,1111,0");
        commands.push_back("MOVEAN,TL,534.411979,-144.54213,159.2,TL,539.834377,-147.997334,159.2,1111,1111,0");
        commands.push_back("MOVEAN,TL,574.080304,-163.307388,159.2,TL,616.746349,-135.865174,159.2,1111,1111,0");
        commands.push_back("MOVELN,TL,616.746349,-135.865174,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,622.517121,-128.33156,160.0,TL,630.952955,-125.281741,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,635.752013,-124.365879,160.0,TL,641.42311,-119.054772,160.0,1111,1111,0");
        commands.push_back("MOVELN,TL,649.752241,-90.329113,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,647.507197,-83.014427,160.0,TL,653.896933,-76.897508,160.0,1111,1111,0");
        commands.push_back("MOVELN,TL,679.796467,-23.434994,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,676.48257,-16.855654,160.0,TL,682.131284,-7.689285,160.0,1111,1111,0");
        commands.push_back("MOVELN,TL,680.890534,47.10447,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,673.969108,58.186618,160.0,TL,679.774399,68.225473,160.0,1111,1111,0");
        commands.push_back("MOVELN,TL,679.097466,88.828563,160.0,1111,1111,0");
        commands.push_back("MOVELN,TL,672.111429,123.968222,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,665.140607,132.824019,160.0,TL,670.704299,142.204112,160.0,1111,1111,0");
        commands.push_back("MOVELN,TL,670.434902,155.048533,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,661.622853,161.51742,160.0,TL,655.874015,168.11615,160.0,1111,1111,0");
        commands.push_back("MOVELN,TL,619.834274,164.813453,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,611.710919,159.408143,160.0,TL,606.651895,162.11787,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,600.398059,167.477713,160.0,TL,594.232296,162.397198,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,587.909183,158.098726,160.0,TL,579.941666,160.970507,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,566.913694,158.357427,160.0,TL,556.918887,165.23554,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,549.35132,168.857285,160.0,TL,542.612342,160.624016,160.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,540.209199,152.503034,160.0,TL,530.327241,149.383785,160.0,1111,1111,0");
        commands.push_back("MOVELN,TL,530.327241,149.383785,159.5,1111,1111,0");
        commands.push_back("MOVELN,TL,499.259665,115.896271,159.5,1111,1111,0");
        commands.push_back("MOVEAN,TL,500.673615,107.592788,159.5,TL,496.506284,98.534183,159.5,1111,1111,0");
        commands.push_back("MOVEAN,TL,505.038024,78.73316,159.5,TL,503.975844,56.618144,159.5,1111,1111,0");
        commands.push_back("MOVEAN,TL,508.219737,49.670791,159.5,TL,499.264022,48.503173,159.5,1111,1111,0");
        commands.push_back("MOVELN,TL,499.264022,48.503173,159.0,1111,1111,0");
        commands.push_back("MOVELN,TL,482.324898,20.237931,159.0,1111,1111,0");
        commands.push_back("MOVELN,TL,479.978328,5.377368,159.0,1111,1111,0");
        commands.push_back("MOVEAN,TL,488.196294,-3.643101,159.0,TL,481.666249,-11.421064,159.0,1111,1111,0");
        commands.push_back("MOVELN,TL,485.38542,-29.183973,159.0,1111,1111,0");
        commands.push_back("MOVELN,TL,485.38542,-29.183973,360,1111,1111,0");
    }
#endif
    void ChinHardwareInterface::write(WhiArmInterface* HwIf, double Dt)
    {
        if (((DriverSocket*)drivers_map_[hw_config_->hardware_].get())->isServoOn(2000))
        {
#ifdef DEBUG
            /// simulate demo.py
            // static int index = 0;
            // if (index == 0)
            // {
            //     initDebugCommandList();
            // }
            // if (index < commands_demo.size())
            // {
            //     drivers_map_[hw_config_->hardware_]->actuate(commands_demo[index]);
            //     ++index;
            // }
            /// single joint testing
            static double step = 0.5;
            static const double LIMIT_MIN = -30.0;
            static const double LIMIT_MAX = 30.0;
            static double pos = LIMIT_MIN;
            std::string cmd = std::string("MOVEJ,1") + "," + std::to_string(pos) + ",5.0,10.0";
            drivers_map_[hw_config_->hardware_]->actuate(cmd);
            pos += step;
            if (pos >= LIMIT_MAX)
            {
                step *= -1.0;
                pos = LIMIT_MAX;
            }
            else if (pos <= LIMIT_MIN)
            {
                step *= -1.0;
                pos = LIMIT_MIN;
            }
#else
            std::string positions;
            for (std::size_t i = 0; i < std::min(hw_config_->forward_dirs_.size(), joint_position_commands_.size()); ++i)
            {
                positions += std::to_string(angles::to_degrees(hw_config_->forward_dirs_[i] * joint_position_commands_[i])) + ",";
            }
            positions.pop_back();
            if (controller_type_.find("position_controllers") != std::string::npos)
            {
                drivers_map_[hw_config_->hardware_]->actuate(composeCommand(positions));
            }
            else if (controller_type_.find("pos_vel_controllers") != std::string::npos)
            {
                std::string angulars;
                std::string accelerations;
                for (std::size_t i = 0; i < joint_velocity_commands_.size(); ++i)
                {
                    double angular = 10.0 * angles::to_degrees(fabs(joint_velocity_commands_[i]));
                    angulars += std::to_string(angular) + ",";
                    accelerations += std::to_string(2.0 * angular) + ",";
                }
                angulars.pop_back();
                accelerations.pop_back();

                drivers_map_[hw_config_->hardware_]->actuate(composeCommand(positions, angulars, accelerations));
            }
            else if (controller_type_.find("pos_vel_acc_controllers") != std::string::npos)
            {
                std::string angulars;
                for (std::size_t i = 0; i < joint_velocity_commands_.size(); ++i)
                {
                    angulars += std::to_string(angles::to_degrees(fabs(joint_velocity_commands_[i]))) + ",";
                }
                angulars.pop_back();
                std::string accelerations;
                for (std::size_t i = 0; i < joint_acceleration_commands_.size(); ++i)
                {
                    accelerations += std::to_string(angles::to_degrees(fabs(joint_acceleration_commands_[i]))) + ",";
                }
                accelerations.pop_back();

                drivers_map_[hw_config_->hardware_]->actuate(composeCommand(positions, angulars, accelerations));
#ifdef DEBUG
                std::cout << "velocity command:" << std::endl;
                for (const auto& it : joint_velocity_commands_)
                {
                    std::cout << it << ",";
                }
                std::cout << std::endl;
#endif
            }
#endif
        }
    }

    std::string ChinHardwareInterface::composeCommand(const std::string& Positions) const
    {
        std::string command("MOVEJ,DOF," + Positions + ",DOF,");
        for (const auto& it : hw_config_->angular_velocities_)
        {
            command += std::to_string(hw_config_->speed_rate_ * it) + ",";
        }
        command += "DOF,";
        for (const auto& it : hw_config_->angular_accelerations_)
        {
            command += std::to_string(it) + ",";
        }
        command += "0";

#ifdef DEBUG
        ROS_INFO_STREAM(command);
#endif
        return command;
    }

    std::string ChinHardwareInterface::composeCommand(const std::string& Positions, const std::string& Velocities,
        const std::string& Accelerations) const
    {
#ifdef DEBUG
        ROS_INFO_STREAM("MOVEJ,DOF," + Positions + ",DOF," + Velocities + ",DOF," + Accelerations + ",0");
#endif
        return std::string("MOVEJ,DOF," + Positions + ",DOF," + Velocities + ",DOF," + Accelerations + ",0");
    }
}
