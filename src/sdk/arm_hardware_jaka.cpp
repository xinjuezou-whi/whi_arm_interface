/******************************************************************
arm hardware interface of JAKA under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- JAKA hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/arm_hardware_jaka.h"
#include "whi_arm_interface/driver_socket_json.h"
#include "whi_arm_interface/jakaAPI/jkerr.h"
#include "whi_arm_interface/jakaAPI/jktypes.h"
#include <json/json.h>
#include <angles/angles.h>

#include <thread>

namespace whi_arm_hardware_interface
{
    using namespace hardware_interface;

    JakaHardwareInterface::JakaHardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : ArmHardware(NodeHandle)
    {
        init();
    }

    JakaHardwareInterface::~JakaHardwareInterface()
    {
        if (jaka_api_instance_)
        {
            jaka_api_close();
        }
        else
        {
            jaka_tcp_close();
        }
    }

    void JakaHardwareInterface::init()
    {
        // joints
        node_handle_->getParam("/jaka_arm/controllers/command/joints", joint_names_);
        if (joint_names_.size() == 0)
        {
            // especially for rosrun mode
            std::string sum;
            for (int i = 0; i < 6; ++i)
            {
                joint_names_.push_back("joint" + std::to_string(i + 1));
                sum += joint_names_.back() + "\n";
            }
            sum.pop_back();
            ROS_WARN((std::string("No joints found on parameter server for controller. Name them with:\n") + sum).c_str());
        }

        // drivers
        node_handle_->param("/whi_arm_interface/hardware", name_, std::string(hardware[SOCKET]));
        if (name_ == hardware[SOCKET])
        {
            std::string addr;
            int port;
            int dataLength;
            node_handle_->param("/whi_arm_interface/socket/addr", addr, std::string("10.5.5.1"));
            node_handle_->param("/whi_arm_interface/socket/port", port, 10001);
            drivers_map_.emplace(name_, std::make_unique<DriverSocketJson>(name_, addr, port));

            jaka_tcp_init();
        }
        else if (name_ == hardware[JAKA_API])
        {
            std::string addr;
            node_handle_->param("/whi_arm_interface/jaka_api/addr", addr, std::string("10.5.5.1"));

            jaka_api_init(addr);
        }
        else
        {
            ROS_ERROR_STREAM("failed to init driver of " << name_);
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
        node_handle_->param("/jaka_arm/controllers/command/type", controller_type_, std::string(""));
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
        controller_manager_ = std::make_unique<controller_manager::ControllerManager>(this, *node_handle_);

        node_handle_->param("/whi_arm_interface/loop_hz", loop_hz_, 10.0);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq, std::bind(&JakaHardwareInterface::update, this, std::placeholders::_1)));
    }

    void JakaHardwareInterface::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void JakaHardwareInterface::read()
    {
        static bool init = true;

        std::vector<double> positions;
        if (jaka_api_instance_)
        {
            positions = jaka_api_readPositions();
        }
        else
        {
            positions = jaka_tcp_readPositions();
        }

        for (std::size_t i = 0; i < std::min(joint_position_.size(), positions.size()); ++i)
        {
            joint_position_[i] = positions[i];
        }
        if (init && !positions.empty())
        {
            joint_position_command_ = joint_position_;
            init = false;
        }
    }

    void JakaHardwareInterface::write(ros::Duration ElapsedTime)
    {
        if (jaka_api_instance_)
        {
            jaka_api_servoPositions(joint_position_command_, ElapsedTime.toSec());
        }
        else
        {
            jaka_tcp_servoPositions(joint_position_command_, ElapsedTime.toSec());
        }
    }

    bool JakaHardwareInterface::jaka_tcp_init()
    {
        bool res = true;

        ((DriverSocketJson*)drivers_map_[name_].get())->setParamsKey(paramKey, PARAM_KEY_SUM);

        Json::Value root;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"servo_move","relFlag":0}
        root["cmdName"] = "servo_move";
        root["relFlag"] = 0;
        requests.push_back(Json::writeString(builder, root));
        // delay 500ms
        requests.push_back("delay:500");
        // {"cmdName":"set_servo_move_filter","filter_type":1",lpf_cf":0.5}
        root.clear();
        root["cmdName"] = "set_servo_move_filter";
        root["filter_type"] = 1;
        root["lpf_cf"] = 0.5;
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"power_on"}
        root.clear();
        root["cmdName"] = "power_on";
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"enable_robot"}
        root.clear();
        root["cmdName"] = "enable_robot";
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"servo_move","relFlag":1}
        root["cmdName"] = "servo_move";
        root["relFlag"] = 1;
        requests.push_back(Json::writeString(builder, root));

        ((DriverSocketJson*)drivers_map_[name_].get())->request(requests);

        return res;
    }

    void JakaHardwareInterface::jaka_tcp_close()
    {
        Json::Value root;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"servo_move","relFlag":0}
        root["cmdName"] = "servo_move";
        root["relFlag"] = 0;
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"disable_robot"}
        root["cmdName"] = "disable_robot";
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"power_off"}
        root["cmdName"] = "power_off";
        requests.push_back(Json::writeString(builder, root));

        ((DriverSocketJson*)drivers_map_[name_].get())->request(requests);
    }

    std::vector<double> JakaHardwareInterface::jaka_tcp_readPositions()
    {
        Json::Value root;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"get_joint_pos"}
        root["cmdName"] = "get_joint_pos";
        requests.push_back(Json::writeString(builder, root));

        ((DriverSocketJson*)drivers_map_[name_].get())->request(requests);
        auto read = ((DriverSocketJson*)drivers_map_[name_].get())->readParam(paramKey[JOINT_POS]);
        for (auto& it : read)
        {
            it = angles::from_degrees(it);
        }
#ifdef DEBUG
        std::cout << "read positions:";
        for (const auto& it : read)
        {
            std::cout << it << ",";
        }
        std::cout << std::endl;
#endif
        return read;
    }

    void JakaHardwareInterface::jaka_tcp_servoPositions(const std::vector<double>& Positions, double Duration)
    {
        int stepNum = int(Duration / 0.008);

        Json::Value root;
        Json::Value data;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"servo_j","relFlag":0,"jointPosition":[0.1,0,0,0,0,0],"stepNum":1}
        root["cmdName"] = "servo_j";
        root["relFlag"] = 0;
        for (const auto& it : Positions)
        {
            root["jointPosition"].append(angles::to_degrees(it));
        }
#ifdef DEBUG
        std::cout << "commanded positions:";
        for (const auto& it : Positions)
        {
            std::cout << it << ",";
        }
        std::cout << "with step:" << stepNum << std::endl;
#endif
        root["stepNum"] = stepNum;
        requests.push_back(Json::writeString(builder, root));

        ((DriverSocketJson*)drivers_map_[name_].get())->request(requests);
    }

    bool JakaHardwareInterface::jaka_api_init(const std::string& Addr)
    {
        bool res = true;
        jaka_api_instance_ = std::make_unique<JAKAZuRobot>();
        if (jaka_api_instance_->login_in(Addr.c_str()) == ERR_SUCC)
        {
            jaka_api_instance_->servo_move_enable(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // set filter parameter
            jaka_api_instance_->servo_move_use_joint_LPF(0.5);

            jaka_api_instance_->power_on();
            jaka_api_instance_->enable_robot();

            jaka_api_instance_->servo_move_enable(true);

            ROS_INFO_STREAM("JAKA driver is initialized successfully");
        }
        else
        {
            jaka_api_instance_ = nullptr;
            res = false;
            ROS_ERROR_STREAM("failed to initialize JAKA driver");
        }

        return res;
    }

    void JakaHardwareInterface::jaka_api_close()
    {
        if (jaka_api_instance_)
        {
            jaka_api_instance_->servo_move_enable(false);
            jaka_api_instance_->disable_robot();
            jaka_api_instance_->power_off();
            jaka_api_instance_->login_out();
        }
    }

    std::vector<double> JakaHardwareInterface::jaka_api_readPositions() const
    {
        RobotStatus status;
        jaka_api_instance_->get_robot_status(&status);

        std::vector<double> positions;
        for (const auto& it : status.joint_position)
        {
            positions.push_back(it);
        }
#ifdef DEBUG
        std::cout << "read positions:";
        for (const auto& it : positions)
        {
            std::cout << it << ",";
        }
        std::cout << std::endl;
#endif

        return positions;
    }

    void JakaHardwareInterface::jaka_api_servoPositions(const std::vector<double>& Positions, double Duration)
    {
        int stepNum = int(Duration / 0.008);

        JointValue positions;
        for (int i = 0; i < std::min(Positions.size(), sizeof(positions.jVal)); ++i)
        {
            positions.jVal[i] = Positions[i];
        }
#ifdef DEBUG
        std::cout << "commanded positions:";
        for (const auto& it : positions.jVal)
        {
            std::cout << it << ",";
        }
        std::cout << "with step:" << stepNum << std::endl;
#endif

        auto res = jaka_api_instance_->servo_j(&positions, MoveMode::ABS, stepNum);
        if (res != ERR_SUCC)
        {
            ROS_WARN_STREAM("failed to execute servo_j motion with error code: " << res);
        }
    }
}
