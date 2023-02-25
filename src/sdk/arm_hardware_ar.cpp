/******************************************************************
arm hardware interface of ar2 under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- ar2 hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/arm_hardware_ar.h"
#include "whi_arm_interface/driver_serial.h"
#include "whi_arm_interface/driver_rosserial.h"

#include <angles/angles.h>
#include <std_msgs/String.h>
#include <thread>

namespace whi_arm_hardware_interface
{
	using namespace hardware_interface;

    ArHardwareInterface::ArHardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : ArmHardware(NodeHandle)
    {
        init();
    }

    void ArHardwareInterface::init()
    {
        ros::NodeHandle nh_private("~");
        bool toHome;
        nh_private.param("home", toHome, false);
        homing_state_ = toHome ? STA_TO_HOME : STA_HOMED;
        nh_private.param("close_loop", mode_close_loop_, true);

        // joints
        node_handle_->getParam("/ar_arm/controllers/command/joints", joint_names_);
        if (joint_names_.size() == 0)
        {
            // especially for rosrun mode
            std::string sum;
            for (int i = 0; i < 6; ++i)
            {
                joint_names_.push_back("joint_" + std::to_string(i + 1));
                sum += joint_names_.back() + "\n";
            }
            sum.pop_back();
            ROS_WARN((std::string("No joints found on parameter server for controller. Name them with:\n") + sum).c_str());
        }
        node_handle_->getParam("/ar_arm/hardware_interface/steps_per_degree/", steps_per_deg_);
        node_handle_->getParam("/ar_arm/hardware_interface/forward_dir/", forward_dir_);
        node_handle_->getParam("/ar_arm/hardware_interface/limits_dir/", limits_dir_);
        node_handle_->getParam("/ar_arm/hardware_interface/home_offsets/", home_offsets_);
        node_handle_->getParam("/ar_arm/hardware_interface/home_kinematics/", home_kinematics_);
        for (std::size_t i = 0; i < joint_names_.size(); ++i)
        {
            // A B C D E F...
            axes_prefix_.push_back(char(65 + i));
        }
        node_handle_->param("/ar_arm/hardware_interface/speed_rate", speed_rate_, 25);
        node_handle_->param("/ar_arm/hardware_interface/acc_duration", acc_duration_, 15);
        node_handle_->param("/ar_arm/hardware_interface/acc_rate", acc_rate_, 10);
        node_handle_->param("/ar_arm/hardware_interface/dec_duration", dec_duration_, 20);
        node_handle_->param("/ar_arm/hardware_interface/dec_rate", dec_rate_, 5);

        // drivers
        std::string hardwareStr;
        node_handle_->param("/ar2_arm/hardware_interface/hardware", hardwareStr, std::string(hardware[ROSSERIAL]));
        if (hardwareStr == hardware[ROSSERIAL])
        {
            std::string topic;
            node_handle_->param("/ar_arm/hardware_interface/rosserial/topic", topic, std::string("/arm_hardware_interface"));
            drivers_map_.emplace(name_, std::make_unique<DriverRosserial>(name_, node_handle_, topic));
            ((DriverRosserial*)drivers_map_[name_].get())->setMotor(limits_dir_);
            ((DriverRosserial*)drivers_map_[name_].get())->registerResponse(std::bind(&ArHardwareInterface::callbackResponse, this, std::placeholders::_1));
        }
        else if (hardwareStr == hardware[SERIAL])
        {
            // currently AR2's arduino accept single combined command,
            // therefore init one serial instance
            std::string port;
            int baudrate = -1;
            if (node_handle_->param("/ar_arm/hardware_interface/serial/port", port, std::string()) &&
                node_handle_->param("/ar_arm/hardware_interface/serial/baudrate", baudrate, -1))
            {
                try
                {
                    auto serialInst = std::make_shared<serial::Serial>(port, baudrate, serial::Timeout::simpleTimeout(500));
                    drivers_map_.emplace(name_, std::make_unique<DriverSerial>(name_, serialInst));
                    ((DriverSerial*)drivers_map_[name_].get())->setMotor(limits_dir_);
                }
                catch (serial::IOException& e)
                {
                    ROS_FATAL_STREAM_NAMED("failed to open serial %s", port.c_str());
                }
            }
            else
            {
                ROS_FATAL_NAMED("failed to get serial params %s, %d", port.c_str(), baudrate);
            }
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
        joint_effort_command_.resize(num_joints_);

        // initialize controller
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            // create joint state interface
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            // create joint command interface: position
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            position_joint_interface_.registerHandle(jointPositionHandle);
        }
        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);

        // controller
        node_handle_->param("/ar_arm/hardware_interface/loop_hz", loop_hz_, 10.0);
        controller_manager_ = std::make_unique<controller_manager::ControllerManager>(this, *node_handle_);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq, std::bind(&ArHardwareInterface::update, this, std::placeholders::_1)));
    }

    void ArHardwareInterface::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void ArHardwareInterface::read()
    {
        // do nothing, since the position is updated by message callback
    }

    void ArHardwareInterface::write(ros::Duration ElapsedTime)
    {
        /// rosserial
        if (homing_state_ == STA_HOMED)
        {
            std::string cmd("MJ");
            for (std::size_t i = 0; i < joint_position_command_.size(); ++i)
            {
                double degCmd = angles::to_degrees(joint_position_command_[i]);
                double degCur = angles::to_degrees(joint_position_[i]);
                int step = int((degCmd - degCur) * steps_per_deg_[i]) * forward_dir_[i];
                cmd.append(std::string(1, axes_prefix_[i]) + (step >= 0 ? "1" : "0") + std::to_string(abs(step)));
            }
            cmd.append(std::string("S") + std::to_string(speed_rate_) +
                "G" + std::to_string(acc_duration_) + "H" + std::to_string(acc_rate_) +
                "I" + std::to_string(dec_duration_) + "K" + std::to_string(dec_rate_));

            drivers_map_[name_]->actuate(cmd);
#ifdef DEBUG
            std::cout << "arduino cmd " << cmd << std::endl;
#endif
            if (!mode_close_loop_)
            {
                // update current to command
                for (std::size_t i = 0; i < joint_position_.size(); ++i)
                {
                    joint_position_[i] = joint_position_command_[i];
                }
            }
        }
        else if (homing_state_ == STA_TO_HOME)
        {
            std::string cmd("hm");
            for (std::size_t i = 0; i < joint_position_command_.size(); ++i)
            {
                int step = int(home_offsets_[i] * steps_per_deg_[i]) * forward_dir_[i];
                cmd.append(std::string(1, axes_prefix_[i]) + (step >= 0 ? "1" : "0") + std::to_string(abs(step)));
            }
            cmd.append(std::string("S") + std::to_string(int(home_kinematics_[0])) +
                "G" + std::to_string(int(home_kinematics_[1])) + "H" + std::to_string(int(home_kinematics_[2])) +
                "I" + std::to_string(int(home_kinematics_[3])) + "K" + std::to_string(int(home_kinematics_[4])) +
                "l");
            for (const auto& it : limits_dir_)
            {
                cmd.append(it > 0 ? "1" : "0");
            }

            drivers_map_[name_]->actuate(cmd);
#ifdef DEBUG
            std::cout << "arduino cmd " << cmd << std::endl;
#endif
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
            if (mode_close_loop_)
            {
                try
                {
                    std::size_t begin = 0;
                    std::size_t end = 0;
                    for (std::size_t i = 0; i < joint_position_.size(); ++i)
                    {
                        begin = State.find('p', begin);
                        end = State.find('p', begin + 1);
                        if (end > begin)
                        {
                            joint_position_[i] = angles::from_degrees(forward_dir_[i] * std::stoi(State.substr(begin + 1, end - begin - 1)) / steps_per_deg_[i]);
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
