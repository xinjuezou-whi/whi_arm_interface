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
        node_handle_->getParam("/chin_arm/hardware_interface/joints", joint_names_);
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
        node_handle_->param("/chin_arm/hardware_interface/speed_rate", speed_rate_, 25);
        node_handle_->param("/chin_arm/hardware_interface/acc_duration", acc_duration_, 15);
        node_handle_->param("/chin_arm/hardware_interface/acc_rate", acc_rate_, 10);
        node_handle_->param("/chin_arm/hardware_interface/dec_duration", dec_duration_, 20);
        node_handle_->param("/chin_arm/hardware_interface/dec_rate", dec_rate_, 5);

        // drivers
        std::string hardwareStr;
        node_handle_->param("/chin_arm/hardware_interface/hardware", hardwareStr, std::string(hardware[SOCKET]));
        if (hardwareStr == hardware[SOCKET])
        {
            std::string addr;
            int port;
            node_handle_->param("/chin_arm/hardware_interface/socket/addr", addr, std::string("192.168.4.44"));
            node_handle_->param("/chin_arm/hardware_interface/socket/port", port, 8888);
            drivers_map_.emplace(name_, std::make_unique<DriverSocket>(name_, addr, port));
            ((DriverSocket*)drivers_map_[name_].get())->setMotor();
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

            // create velocity joint interface
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            position_joint_interface_.registerHandle(jointPositionHandle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);

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
        drivers_map_[name_]->readAngle();
    }

    void ChinHardwareInterface::write(ros::Duration ElapsedTime)
    {
        ((DriverSocket*)drivers_map_[name_].get())->getState();
    }
}
