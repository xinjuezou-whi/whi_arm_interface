/******************************************************************
arm hardware interface of chin under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- chin hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/arm_hardware_chin.h"
#include "whi_arm_interface/driver_socket.h"

#include <angles/angles.h>

namespace whi_arm_hardware_interface
{
    using namespace hardware_interface;

    ChinHardwareInterface::ChinHardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : ArmHardware(NodeHandle)
    {
        init();
    }

    void ChinHardwareInterface::init()
    {
        ros::NodeHandle nh_private("~");
        bool toHome;
        nh_private.param("home", toHome, false);
        homing_state_ = toHome ? STA_TO_HOME : STA_HOMED;

        // joints
        node_handle_->getParam("/chin_arm/controllers/command/joints", joint_names_);
        if (joint_names_.size() == 0)
        {
            // especially for rosrun mode
            std::string sum;
            for (int i = 0; i < 6; ++i)
            {
                joint_names_.push_back("chin_joint" + std::to_string(i + 1));
                sum += joint_names_.back() + "\n";
            }
            sum.pop_back();
            ROS_WARN((std::string("No joints found on parameter server for controller. Name them with:\n") + sum).c_str());
        }

        // drivers
        node_handle_->getParam("/whi_arm/hardware_interface/forward_dirs", forward_dirs_);
        node_handle_->param("/whi_arm/hardware_interface/speed_rate", speed_rate_, 50.0);
        speed_rate_ /= 100.0;
        node_handle_->getParam("/whi_arm/hardware_interface/angular_velocities", angulars_);
        node_handle_->getParam("/whi_arm/hardware_interface/angular_accelerations", accelerations_);
        std::string hardwareStr;
        node_handle_->param("/whi_arm/hardware_interface/hardware", hardwareStr, std::string(hardware[SOCKET]));
        if (hardwareStr == hardware[SOCKET])
        {
            std::string addr;
            int port;
            int dataLength;
            node_handle_->param("/whi_arm/hardware_interface/socket/addr", addr, std::string("192.168.4.44"));
            node_handle_->param("/whi_arm/hardware_interface/socket/port", port, 8888);
            node_handle_->param("/whi_arm/hardware_interface/socket/response_length", dataLength, 128);
            drivers_map_.emplace(name_, std::make_unique<DriverSocket>(name_, addr, port));
            ((DriverSocket*)drivers_map_[name_].get())->setMotor(dataLength);
        }
        else
        {
            ROS_FATAL_STREAM_NAMED("failed to init driver of %s", hardwareStr.c_str());
        }

        // resize vectors
        num_joints_ = joint_names_.size();
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_acceleration_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // initialize controller
        node_handle_->param("/chin_arm/controllers/command/type", controller_type_, std::string(""));
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            // create joint state interface
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            if (controller_type_.find("position_controllers") != std::string::npos)
            {
                // create joint command interface: position
                JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
                position_joint_interface_.registerHandle(jointPositionHandle);
            }
            else if (controller_type_.find("pos_vel_controllers") != std::string::npos)
            {
                // create joint command interface: position, velocity
                PosVelJointHandle jointPosVelHandle(jointStateHandle,
                    &joint_position_command_[i], &joint_velocity_command_[i]);
                pos_vel_joint_interface_.registerHandle(jointPosVelHandle);               
            }
            else if (controller_type_.find("pos_vel_acc_controllers") != std::string::npos)
            {
                // create joint command interface: position, velocity, acceleration
                PosVelAccJointHandle jointPosVelAccHandle(jointStateHandle,
                    &joint_position_command_[i], &joint_velocity_command_[i], &joint_acceleration_command_[i]);
                pos_vel_acc_joint_interface_.registerHandle(jointPosVelAccHandle);
            }
        }
        registerInterface(&joint_state_interface_);
        if (controller_type_.find("position_controllers") != std::string::npos)
        {
            registerInterface(&position_joint_interface_);
        }
        else if (controller_type_.find("pos_vel_controllers") != std::string::npos)
        {
            registerInterface(&pos_vel_joint_interface_);
        }
        else if (controller_type_.find("pos_vel_acc_controllers") != std::string::npos)
        {
            registerInterface(&pos_vel_acc_joint_interface_);
        }

        // controller
        node_handle_->param("/whi_arm/hardware_interface/loop_hz", loop_hz_, 10.0);
        controller_manager_ = std::make_unique<controller_manager::ControllerManager>(this, *node_handle_);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq, std::bind(&ChinHardwareInterface::update, this, std::placeholders::_1)));
    }

    void ChinHardwareInterface::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void ChinHardwareInterface::read()
    {
        if (((DriverSocket*)drivers_map_[name_].get())->isServoOn(3000))
        {
            std::vector<std::string> params { "ANGLES", "DANGLES" };
            ((DriverSocket*)drivers_map_[name_].get())->request(params);
            std::vector<double> angles = ((DriverSocket*)drivers_map_[name_].get())->readParam(params[0]);
            for (std::size_t i = 0; i < 
                std::min(std::min(joint_position_.size(), angles.size()), forward_dirs_.size()); ++i)
            {
                joint_position_[i] = angles::from_degrees(forward_dirs_[i] * angles[i]);
            }
            std::vector<double> velocities = ((DriverSocket*)drivers_map_[name_].get())->readParam(params[1]);
            for (std::size_t i = 0; i < 
                std::min(std::min(joint_velocity_.size(), velocities.size()), forward_dirs_.size()); ++i)
            {
                joint_velocity_[i] = angles::from_degrees(forward_dirs_[i] * velocities[i]);
            }
#ifdef DEBUG
            std::cout << "velocity read" << std::endl;
            for (const auto& it : joint_velocity_)
            {
                std::cout << it << ",";
            }
            std::cout << std::endl;
#endif
            // there's no acceleration data available in joint state handle
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
    void ChinHardwareInterface::write(ros::Duration ElapsedTime)
    {
        if (((DriverSocket*)drivers_map_[name_].get())->isServoOn(2000))
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
            //     drivers_map_[name_]->actuate(commands_demo[index]);
            //     ++index;
            // }
            /// single joint testing
            static double step = 0.5;
            static const double LIMIT_MIN = -30.0;
            static const double LIMIT_MAX = 30.0;
            static double pos = LIMIT_MIN;
            std::string cmd = std::string("MOVEJ,1") + "," + std::to_string(pos) + ",5.0,10.0";
            drivers_map_[name_]->actuate(cmd);
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
            for (std::size_t i = 0; i < std::min(forward_dirs_.size(), joint_position_command_.size()); ++i)
            {
                positions += std::to_string(angles::to_degrees(forward_dirs_[i] * joint_position_command_[i])) + ",";
            }
            positions.pop_back();
            if (controller_type_.find("position_controllers") != std::string::npos)
            {
                drivers_map_[name_]->actuate(composeCommand(positions));
            }
            else if (controller_type_.find("pos_vel_controllers") != std::string::npos)
            {
                std::string angulars;
                std::string accelerations;
                for (std::size_t i = 0; i < joint_velocity_command_.size(); ++i)
                {
                    double angular = 10.0 * angles::to_degrees(fabs(joint_velocity_command_[i]));
                    angulars += std::to_string(angular) + ",";
                    accelerations += std::to_string(2.0 * angular) + ",";
                }
                angulars.pop_back();
                accelerations.pop_back();

                drivers_map_[name_]->actuate(composeCommand(positions, angulars, accelerations));
            }
            else if (controller_type_.find("pos_vel_acc_controllers") != std::string::npos)
            {
                std::string angulars;
                for (std::size_t i = 0; i < joint_velocity_command_.size(); ++i)
                {
                    angulars += std::to_string(angles::to_degrees(fabs(joint_velocity_command_[i]))) + ",";
                }
                angulars.pop_back();
                std::string accelerations;
                for (std::size_t i = 0; i < joint_acceleration_command_.size(); ++i)
                {
                    accelerations += std::to_string(angles::to_degrees(fabs(joint_acceleration_command_[i]))) + ",";
                }
                accelerations.pop_back();

                drivers_map_[name_]->actuate(composeCommand(positions, angulars, accelerations));
#ifdef DEBUG
                std::cout << "velocity command:" << std::endl;
                for (const auto& it : joint_velocity_command_)
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
        for (const auto& it : angulars_)
        {
            command += std::to_string(speed_rate_ * it) + ",";
        }
        command += "DOF,";
        for (const auto& it : accelerations_)
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
