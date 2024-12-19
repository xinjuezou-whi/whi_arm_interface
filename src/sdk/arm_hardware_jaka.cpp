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
#include "whi_interfaces/WhiMotionState.h"
#include <json/json.h>
#include <angles/angles.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager_msgs/ListControllers.h>

#include <thread>

namespace whi_arm_hardware_interface
{
    using namespace hardware_interface;

    JakaHardwareInterface::JakaHardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : ArmHardware(NodeHandle)
    {
        init();
    }

    void JakaHardwareInterface::quit()
    {
        // give time to thirdparty dependencies
        std::this_thread::sleep_for(std::chrono::milliseconds(shutdown_patience_));

        if (name_ == hardware[JAKA_API])
        {
            jaka_api_close();
        }
        else if (name_ == hardware[SOCKET])
        {
            jaka_tcp_close();
        }
    }

    JakaHardwareInterface::~JakaHardwareInterface()
    {
        quit();
    }

    static bool ping(const std::string& Addr)
    {
        std::string cmd(std::string("ping ") + Addr + " -w 2");
        int res = system(cmd.c_str());

        return res == 0;
    }

    void JakaHardwareInterface::init()
    {
        // general params
        node_handle_->param("shutdown_patience", shutdown_patience_, 0);
std::cout << "ddddddddddddddddddddddddd " << shutdown_patience_ << std::endl;

        // joints
        node_handle_->getParam("joints", joint_names_);
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
        node_handle_->param("startup_duration", startup_duration_, 10);
        node_handle_->param("velocity_scale", velocity_scale_, 1.0);
        node_handle_->param("payload_weight", payload_weight_, 0.0);
        if (node_handle_->getParam("payload_to_tcp", payload_to_tcp_))
        {
            for (auto& it : payload_to_tcp_)
            {
                it *= 1000.0; // JAKA requires mm
            }
        }
        else
        {
            payload_to_tcp_.resize(3);
        }
        node_handle_->param("socket/addr", addr_, std::string("10.5.5.1"));
        node_handle_->param("hardware", name_, std::string(hardware[SOCKET]));
        node_handle_->param("loop_hz", loop_hz_, 10.0);
        initializing();

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
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            // create joint state interface
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            // create joint command interface: position
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            position_joint_interface_.registerHandle(jointPositionHandle);
            scaled_controllers::ScaledJointHandle scaledPosJointHandle(jointStateHandle, &joint_position_command_[i], &velocity_scale_);
            scaled_position_joint_interface_.registerHandle(scaledPosJointHandle);

            // create joint command interface: position, velocity
            PosVelJointHandle jointPosVelHandle(jointStateHandle,
                &joint_position_command_[i], &joint_velocity_command_[i]);
            pos_vel_joint_interface_.registerHandle(jointPosVelHandle);               

            // create joint command interface: position, velocity, acceleration
            PosVelAccJointHandle jointPosVelAccHandle(jointStateHandle,
                &joint_position_command_[i], &joint_velocity_command_[i], &joint_acceleration_command_[i]);
            pos_vel_acc_joint_interface_.registerHandle(jointPosVelAccHandle);

            // create joint command interface: velocity
            JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
            velocity_joint_interface_.registerHandle(jointVelocityHandle);
            scaled_controllers::ScaledJointHandle scaledVelJointHandle(jointStateHandle, &joint_velocity_command_[i], &velocity_scale_);
            scaled_velocity_joint_interface_.registerHandle(scaledVelJointHandle);
        }
        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&scaled_position_joint_interface_);
        registerInterface(&pos_vel_joint_interface_);
        registerInterface(&pos_vel_acc_joint_interface_);
        registerInterface(&velocity_joint_interface_);
        registerInterface(&scaled_velocity_joint_interface_);

        // controller
        controller_manager_ = std::make_unique<controller_manager::ControllerManager>(this, *node_handle_);
        client_controller_manager_ = std::make_unique<ros::ServiceClient>(
            node_handle_->serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers"));

        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(
            updateFreq, std::bind(&JakaHardwareInterface::update, this, std::placeholders::_1)));
    }

    void JakaHardwareInterface::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        if (!is_protective_)
        {
            write(elapsed_time_);
        }
    }

    void JakaHardwareInterface::read()
    {
        if (initialized_)
        {
            static bool first = true;

            bool res = true;
            if (name_ == hardware[JAKA_API])
            {
                res = jaka_api_read();
                // only get_robot_status is multi-thread safe, therefore taking synchronous mech
                is_protective_ = jaka_api_isProtective();
            }
            else if (name_ == hardware[SOCKET])
            {
                res = jaka_tcp_read();
                is_protective_ = jaka_tcp_isProtective();
            }

            if (first && res)
            {
                joint_position_command_ = joint_position_;
                first = false;
                standby_ = true;
            }

            if (is_protective_)
            {
                // get the controller name through service
                std::string controllerName("controllers/scaled_pos_controller");
                controller_manager_msgs::ListControllers srv;
                if (client_controller_manager_->call(srv))
                {
                    for (const auto& it : srv.response.controller)
                    {
                        if (it.state == "running" && !it.claimed_resources.front().resources.empty())
                        {
                            controllerName.assign(it.name);
                        }
                    }
                }
                // abort the goal with preemption policy
                auto pub_preempt = std::make_unique<ros::Publisher>(
                    node_handle_->advertise<trajectory_msgs::JointTrajectory>(controllerName + "/command", 1));
                trajectory_msgs::JointTrajectory preempt;
                pub_preempt->publish(preempt);

                whi_interfaces::WhiMotionState msg;
                msg.state = whi_interfaces::WhiMotionState::STA_FAULT;
                if (pub_motion_state_)
                {
                    pub_motion_state_->publish(msg);
                }
                ROS_ERROR_STREAM("arm entered protective state");

                if (name_ == hardware[JAKA_API])
                {
                    jaka_api_protectiveRecover();
                }
                else if (name_ == hardware[SOCKET])
                {
                    jaka_tcp_protectiveRecover();
                }
            }
            else
            {
                whi_interfaces::WhiMotionState msg;
                msg.state = whi_interfaces::WhiMotionState::STA_STANDBY;
                if (pub_motion_state_)
                {
                    pub_motion_state_->publish(msg);
                }
            }
        }
    }

    void JakaHardwareInterface::write(ros::Duration ElapsedTime)
    {
        if (standby_)
        {
            if (name_ == hardware[JAKA_API])
            {
                jaka_api_servoPositions(joint_position_command_, ElapsedTime.toSec());
            }
            else if (name_ == hardware[SOCKET])
            {
                jaka_tcp_servoPositions(joint_position_command_, ElapsedTime.toSec());
            }
        }
    }

    void JakaHardwareInterface::initializing()
    {
        while (!ping(addr_))
        {
            ROS_WARN_STREAM("failed to ping:" << addr_ << ", attempt to another try in " << startup_duration_ << " seconds");
            std::this_thread::sleep_for(std::chrono::seconds(startup_duration_));
        }

        node_handle_->param("lpf", lpf_, 0.5);
        
        bool res = false;
        while (!res)
        {
            if (name_ == hardware[JAKA_API])
            {
                jaka_api_instance_ = std::make_unique<JAKAZuRobot>();
                res = jaka_api_instance_->login_in(addr_.c_str()) == ERR_SUCC;
            }
            else if (name_ == hardware[SOCKET])
            {
                drivers_map_[name_] = std::make_unique<DriverSocketJson>(name_, addr_, 10001);
                bool printTcpFeedback;
                node_handle_->param("debug/print_tcp_feedback", printTcpFeedback, false);
                drivers_map_[name_]->set_debug_params(std::map<std::string, bool>
                    {{ "print_tcp_feedback", printTcpFeedback }});
                ((DriverSocketJson*)drivers_map_[name_].get())->setParamsKey(paramKey, PARAM_KEY_SUM);
                res = jaka_tcp_state();
                if (!res)
                {
                    jaka_tcp_close();
                    drivers_map_[name_]->close();
                }
            }

            if (!res)
            {
                ROS_WARN_STREAM("failed to setup connection, attempt to another try in " << startup_duration_ << " seconds");
                std::this_thread::sleep_for(std::chrono::seconds(startup_duration_));
            }
        }

        while (!initialized_)
        {
            if (name_ == hardware[JAKA_API])
            {
                initialized_ = jaka_api_init(addr_);
            }
            else if (name_ == hardware[SOCKET])
            {
                initialized_ = jaka_tcp_init();
            }

            if (!initialized_)
            {
                ROS_WARN_STREAM("failed to initialize, attempt to another try in " << startup_duration_ << " seconds");
                std::this_thread::sleep_for(std::chrono::seconds(startup_duration_));
            }
        }

        if (initialized_)
        {
            makeOffers();
        }
    }

    bool JakaHardwareInterface::jaka_tcp_init()
    {
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
        // {"cmdName":"set_servo_move_filter","filter_type":1,"lpf_cf":0.5}
        root["cmdName"] = "set_servo_move_filter";
        root["filter_type"] = 1;
        root["lpf_cf"] = lpf_;
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"rapid_rate","rate_value":1.0}
        root["cmdName"] = "rapid_rate";
        root["rate_value"] = velocity_scale_;
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"set_tool_payload","mass":weight,"centroid":[x,y,z]}
        root["cmdName"] = "set_tool_payload";
        root["mass"] = payload_weight_;
        for (const auto& it : payload_to_tcp_)
        {
            root["centroid"].append(it);
        }
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"power_on"}
        root["cmdName"] = "power_on";
        requests.push_back(Json::writeString(builder, root));
        // delay 500ms
        requests.push_back("delay:500");
        // {"cmdName":"enable_robot"}
        root["cmdName"] = "enable_robot";
        requests.push_back(Json::writeString(builder, root));
        // delay 500ms
        requests.push_back("delay:500");
        // {"cmdName":"servo_move","relFlag":1}
        root["cmdName"] = "servo_move";
        root["relFlag"] = 1;
        requests.push_back(Json::writeString(builder, root));
        // delay 500ms
        requests.push_back("delay:500");

        return ((DriverSocketJson*)drivers_map_[name_].get())->request(requests).empty();
    }

    bool JakaHardwareInterface::jaka_tcp_close()
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

        return ((DriverSocketJson*)drivers_map_[name_].get())->request(requests).empty();
    }

    bool JakaHardwareInterface::jaka_tcp_state()
    {
        Json::Value root;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"get_robot_state"}
        root["cmdName"] = "get_robot_state";
        requests.push_back(Json::writeString(builder, root));

        return ((DriverSocketJson*)drivers_map_[name_].get())->request(requests).empty();
    }

    bool JakaHardwareInterface::jaka_tcp_read()
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
        if (!read.empty())
        {
            for (std::size_t i = 0; i < std::min(joint_position_.size(), read.size()); ++i)
            {
                joint_position_[i] = angles::from_degrees(read[i]);
            }

#ifdef DEBUG
            std::cout << "read positions:";
            for (const auto& it : joint_position_)
            {
                std::cout << it << ",";
            }
            std::cout << std::endl;
#endif

            return true;
        }
        else
        {
            return false;
        }
    }

    bool JakaHardwareInterface::jaka_tcp_servoPositions(const std::vector<double>& Positions, double Duration)
    {
        int stepNum = int(Duration / 0.008);
        stepNum = (stepNum > 1e5 || stepNum == 0) ? 1 : stepNum;

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

        auto res = ((DriverSocketJson*)drivers_map_[name_].get())->request(requests);
        if (!res.empty())
        {
            ROS_WARN_STREAM("failed to execute servo_j motion");
        }
        return res.empty();
    }

    bool JakaHardwareInterface::jaka_tcp_setIo(int Addr, int Level)
    {
        Json::Value root;
        Json::Value data;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"set_digital_output","type":0,"index":1,"value":1}
        root["cmdName"] = "set_digital_output";
        root["type"] = 0; // 0 stands for IO on controller
        root["index"] = Addr - 1;
        root["value"] = Level;
        requests.push_back(Json::writeString(builder, root));

        auto res = ((DriverSocketJson*)drivers_map_[name_].get())->request(requests);
        if (!res.empty())
        {
            ROS_WARN_STREAM("failed to execute set digital output");
        }
        return res.empty();
    }

    bool JakaHardwareInterface::jaka_tcp_isProtective()
    {
        Json::Value root;
        Json::Value data;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"protective_stop_status"}
        root["cmdName"] = "protective_stop_status";
        requests.push_back(Json::writeString(builder, root));

        if (((DriverSocketJson*)drivers_map_[name_].get())->request(requests).empty())
        {
            auto read = ((DriverSocketJson*)drivers_map_[name_].get())->readParamStr(paramKey[PROTECTIVE_STOP]);
            if (read.empty())
            {
                return false;
            }
            else
            {
                return read.front() == "1";
            }
        }
        else
        {
            ROS_WARN_STREAM("failed to requiry protective state");
            return false;
        }
    }

    bool JakaHardwareInterface::jaka_tcp_protectiveRecover()
    {
        Json::Value root;
        Json::Value data;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"clear_error"}
        root["cmdName"] = "clear_error";
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"servo_move","relFlag":1}
        root["cmdName"] = "servo_move";
        root["relFlag"] = 1;
        requests.push_back(Json::writeString(builder, root));

        auto res = ((DriverSocketJson*)drivers_map_[name_].get())->request(requests);
        if (!res.empty())
        {
            ROS_WARN_STREAM("failed to recover from protective state");
        }
        return res.empty();
    }

    bool JakaHardwareInterface::jaka_api_init(const std::string& Addr)
    {
        bool res = true;
        jaka_api_instance_ = std::make_unique<JAKAZuRobot>();
        if (jaka_api_instance_->login_in(Addr.c_str()) != ERR_SUCC)
        {
            ROS_ERROR_STREAM("failed to login JAKA driver. failed to initialize JAKA driver");
            jaka_api_instance_ = nullptr;
            return false;
        }
        if (jaka_api_instance_->servo_move_enable(false) != ERR_SUCC)
        {
            ROS_ERROR_STREAM("failed to disable servo mode. failed to initialize JAKA driver");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (jaka_api_instance_->servo_move_use_joint_LPF(lpf_) != ERR_SUCC)
        {
            ROS_ERROR_STREAM("failed to set LPF to " << lpf_ << " Hz. failed to initialize JAKA driver");
            return false;
        }
        if (jaka_api_instance_->set_rapidrate(velocity_scale_) != ERR_SUCC)
        {
            ROS_ERROR_STREAM("failed to set velocity scale to " << velocity_scale_ << ". failed to initialize JAKA driver");
            return false;
        }
        PayLoad payload;
        payload.mass = payload_weight_;
        payload.centroid.x = payload_to_tcp_[0];
        payload.centroid.y = payload_to_tcp_[1];
        payload.centroid.z = payload_to_tcp_[2];
        if (jaka_api_instance_->set_payload(&payload) != ERR_SUCC)
        {
            ROS_ERROR_STREAM("failed to set payload. failed to initialize JAKA driver");
            return false;
        }
        if (jaka_api_instance_->power_on() != ERR_SUCC)
        {
            ROS_ERROR_STREAM("failed to power on arm. failed to initialize JAKA driver");
            return false;
        }
        if (jaka_api_instance_->enable_robot() != ERR_SUCC)
        {
            ROS_ERROR_STREAM("failed to enable arm. failed to initialize JAKA driver");
            return false;
        }
        if (jaka_api_instance_->servo_move_enable(true) != ERR_SUCC)
        {
            ROS_ERROR_STREAM("failed to enable servo mode. failed to initialize JAKA driver");
            return false;
        }

        ROS_INFO_STREAM("JAKA driver is initialized successfully");
        return true;

        // if (jaka_api_instance_->login_in(Addr.c_str()) == ERR_SUCC)
        // {
        //     jaka_api_instance_->servo_move_enable(false);
        //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //     // set filter parameter
        //     jaka_api_instance_->servo_move_use_joint_LPF(5);
        //     jaka_api_instance_->set_rapidrate(velocity_scale_);
        //     PayLoad payload;
        //     payload.mass = payload_weight_;
        //     payload.centroid.x = payload_to_tcp_[0];
        //     payload.centroid.y = payload_to_tcp_[1];
        //     payload.centroid.z = payload_to_tcp_[2];
        //     jaka_api_instance_->set_payload(&payload);
        //     jaka_api_instance_->power_on();
        //     jaka_api_instance_->enable_robot();
        //     jaka_api_instance_->servo_move_enable(true);

        //     ROS_INFO_STREAM("JAKA driver is initialized successfully");
        // }
        // else
        // {
        //     jaka_api_instance_ = nullptr;
        //     res = false;
        //     ROS_ERROR_STREAM("failed to initialize JAKA driver");
        // }

        // return res;
    }

    void JakaHardwareInterface::jaka_api_close()
    {
        jaka_api_instance_->servo_move_enable(false);
        jaka_api_instance_->disable_robot();
        jaka_api_instance_->power_off();
        jaka_api_instance_->login_out();
    }

    bool JakaHardwareInterface::jaka_api_read()
    {
        JointValue jointPos;
        if (jaka_api_instance_->get_joint_position(&jointPos) == ERR_SUCC)
        {
            for (std::size_t i = 0; i < std::min(joint_position_.size(), sizeof(jointPos.jVal)); ++i)
            {
                joint_position_[i] = jointPos.jVal[i];
            }

#ifdef DEBUG
            std::cout << "read positions:";
            for (const auto& it : joint_position_)
            {
                std::cout << it << ",";
            }
            std::cout << std::endl;
#endif
            return true;
        }
        else
        {
            return false;
        }
    }

    bool JakaHardwareInterface::jaka_api_servoPositions(const std::vector<double>& Positions, double Duration)
    {
        int stepNum = int(Duration / 0.008);
        stepNum = (stepNum > 1e5 || stepNum == 0) ? 1 : stepNum;

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

        return res == ERR_SUCC;
    }

    bool JakaHardwareInterface::jaka_api_setIo(int Addr, int Level)
    {
        return jaka_api_instance_->set_digital_output(IO_CABINET, Addr - 1, Level) == ERR_SUCC;
    }

    bool JakaHardwareInterface::jaka_api_isProtective()
    {
        BOOL res = FALSE;
        jaka_api_instance_->is_in_collision(&res);

        return res;
    }

    bool JakaHardwareInterface::jaka_api_protectiveRecover()
    {
        return jaka_api_instance_->collision_recover() == ERR_SUCC &&
            jaka_api_instance_->servo_move_enable(true) == ERR_SUCC;
    }

    void JakaHardwareInterface::makeOffers()
    {
        // advertise arm ready service
        server_ready_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("arm_ready", &JakaHardwareInterface::onServiceReady, this));            
        // advertise io service
        server_io_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("arm_io", &JakaHardwareInterface::onServiceIo, this));
        // create state publisher
        pub_motion_state_ = std::make_unique<ros::Publisher>(
            node_handle_->advertise<whi_interfaces::WhiMotionState>("arm_motion_state", 1));
    }

    bool JakaHardwareInterface::onServiceReady(std_srvs::Trigger::Request& Request, std_srvs::Trigger::Response& Response)
    {
        return (Response.success = standby_);
    }

    bool JakaHardwareInterface::onServiceIo(whi_interfaces::WhiSrvIo::Request& Request,
        whi_interfaces::WhiSrvIo::Response& Response)
    {
        if (Request.addr < 1 || Request.addr > 7)
        {
            Response.result = false;
        }
        else
        {
            if (Request.operation == whi_interfaces::WhiSrvIo::Request::OPER_READ)
            {
                Response.result = false;
            }
            else if (Request.operation == whi_interfaces::WhiSrvIo::Request::OPER_WRITE)
            {
                if (name_ == hardware[JAKA_API])
                {
                    Response.result = jaka_api_setIo(Request.addr, Request.level);
                }
                else if (name_ == hardware[SOCKET])
                {
                    Response.result = jaka_tcp_setIo(Request.addr, Request.level);
                }
            }
        }

        return Response.result;
    }
}
