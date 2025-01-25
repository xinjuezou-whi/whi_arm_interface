/******************************************************************
arm hardware interface of Fair under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- FAIR hardware interfaces
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/arm_hardware_fair.h"
#include "whi_arm_interface/driver_socket_fair.h"
#include "whi_arm_interface/fairAPI/robot_error.h"
#include "whi_arm_interface/fairAPI/robot_types.h"
#include "whi_interfaces/WhiMotionState.h"
#include <json/json.h>
#include <angles/angles.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager_msgs/ListControllers.h>

#include <thread>
#include <regex>

namespace whi_arm_hardware_interface
{
    using namespace hardware_interface;

    FairHardwareInterface::FairHardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : ArmHardware(NodeHandle)
    {
        init();
    }

    void FairHardwareInterface::quit()
    {
        // give time to thirdparty dependencies
        std::this_thread::sleep_for(std::chrono::milliseconds(shutdown_patience_));

        if (name_ == hardware[API])
        {
            api_close();
        }
        else if (name_ == hardware[SOCKET])
        {
            tcp_close();
        }
    }

    FairHardwareInterface::~FairHardwareInterface()
    {
        quit();
    }

    static bool ping(const std::string& Addr)
    {
        std::string cmd(std::string("ping ") + Addr + " -w 2");
        int res = system(cmd.c_str());

        return res == 0;
    }

    void FairHardwareInterface::init()
    {
        // general params
        node_handle_->param("shutdown_patience", shutdown_patience_, 0);

        // joints
        node_handle_->getParam("joints", joint_names_);
        if (joint_names_.size() == 0)
        {
            // especially for rosrun mode
            std::string sum;
            for (int i = 0; i < 6; ++i)
            {
                joint_names_.push_back("j" + std::to_string(i + 1));
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
                it *= 1000.0; // FAIR requires mm
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
            updateFreq, std::bind(&FairHardwareInterface::update, this, std::placeholders::_1)));
    }

    void FairHardwareInterface::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        if (!is_protective_)
        {
            write(elapsed_time_);
        }
    }

    void FairHardwareInterface::read()
    {
        if (initialized_)
        {
            static bool first = true;

            bool res = true;
            if (name_ == hardware[API])
            {
                res = api_read();
                // only get_robot_status is multi-thread safe, therefore taking synchronous mech
                is_protective_ = api_isProtective();
            }
            else if (name_ == hardware[SOCKET])
            {
                res = tcp_read();
                is_protective_ = tcp_isProtective();
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

                if (name_ == hardware[API])
                {
                    api_protectiveRecover();
                }
                else if (name_ == hardware[SOCKET])
                {
                    tcp_protectiveRecover();
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

    void FairHardwareInterface::write(ros::Duration ElapsedTime)
    {
        if (standby_)
        {
            if (name_ == hardware[API])
            {
                api_servoPositions(joint_position_command_, ElapsedTime.toSec());
            }
            else if (name_ == hardware[SOCKET])
            {
                tcp_servoPositions(joint_position_command_, ElapsedTime.toSec());
            }
        }
    }

    void FairHardwareInterface::initializing()
    {
        while (!ping(addr_))
        {
            ROS_WARN_STREAM("failed to ping:" << addr_ << ", attempt to another try in " << startup_duration_ << " seconds");
            std::this_thread::sleep_for(std::chrono::seconds(startup_duration_));
        }
        
        bool res = false;
        while (!res)
        {
            if (name_ == hardware[API])
            {
                // if (!api_instance_)
                // {
                //     api_instance_ = std::make_unique<whi_fair::FRRobot>();
                // }
                // res = (api_instance_->RPC(addr_.c_str()) == ERR_SUCCESS);
                // char version[64] = {0};
                // api_instance_->GetSDKVersion(version);
                // if (!res)
                // {
                //     ROS_ERROR_STREAM("failed to instance FAIR SDK, please check the version: " << version);
                // }
            }
            else if (name_ == hardware[SOCKET])
            {
                drivers_map_[name_] = std::make_unique<DriverSocketFair>(name_, addr_, 8080);
                bool printTcpFeedback;
                node_handle_->param("debug/print_tcp_feedback", printTcpFeedback, false);
                drivers_map_[name_]->set_debug_params(std::map<std::string, bool>
                    {{ "print_tcp_feedback", printTcpFeedback }});

                res = tcp_state();
                if (!res)
                {
                    tcp_close();
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
            if (name_ == hardware[API])
            {
                initialized_ = api_init(addr_);
            }
            else if (name_ == hardware[SOCKET])
            {
                initialized_ = tcp_init();
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

    std::string FairHardwareInterface::packData(const std::string& Command) const
    {
        if (auto search = CMD_MAP_.find(Command); search != CMD_MAP_.end())
        {
            auto index = std::to_string(std::distance(CMD_MAP_.begin(), search));

            return std::string("/f/bIII" + index + "III" + std::to_string(search->second) + "III" +
                std::to_string(search->first.length() + 2) + "III" + search->first + "()III/b/f");
        }
        else
        {
            return std::string();
        }
    }

    static std::vector<std::string> split(const std::string& SrcStr, const std::string& RegexStr)
    {
        std::regex regexz(RegexStr);
        return { std::sregex_token_iterator(SrcStr.begin(), SrcStr.end(), regexz, -1),
            std::sregex_token_iterator() };
    }

    bool FairHardwareInterface::parseFeedback(const std::string& Feedback, int& Index, int& ID, std::vector<std::string>& Data) const
    {
        if (Feedback.empty())
        {
            return false;
        }
        else
        {
            auto posBegin = Feedback.find_first_of("/f/bIII");
            if (posBegin != std::string::npos)
            {
                posBegin += 4;
                for (int i = 0; i < 4; ++i)
                {
                    posBegin += 3;
                    auto posEnd = Feedback.find_first_of("III", posBegin);
                    if (posEnd != std::string::npos)
                    {
                        if (i == 0)
                        {
                            Index = std::stoi(Feedback.substr(posBegin, posEnd - posBegin));
                        }
                        else if (i == 1)
                        {
                            ID = std::stoi(Feedback.substr(posBegin, posEnd - posBegin));
                        }
                        else if (i == 3)
                        {
                            auto contents = Feedback.substr(posBegin, posEnd - posBegin);
                            if (contents.find("errcode") == std::string::npos)
                            {
                                Data = split(contents, ",");
                                return true;
                            }
                            else
                            {
                                return false;
                            }
                        }

                        posBegin = posEnd;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                return false;
            }
        }
    }

    bool FairHardwareInterface::tcp_init()
    {
        std::vector<std::string> requests;

        std::vector<int> paramsEnable{ 0 };
        requests.push_back(packData<int>("RobotEnable", paramsEnable));

        // delay 200ms
        requests.push_back("delay:200");

        std::vector<std::vector<int>> paramsCollision{ {0}, {1, 1, 1, 1, 1, 1}, {0} };
        requests.push_back(packData<std::vector<int>>("SetAnticollision", paramsCollision));

        std::vector<int> paramsMode{ 0 };
        requests.push_back(packData<int>("Mode", paramsMode));

        std::vector<int> paramsSpeed{ int(velocity_scale_ * 100.0) };
        requests.push_back(packData<int>("SetSpeed", paramsSpeed));

        std::vector<std::string> paramsWeight{ "0", std::to_string(payload_weight_) };
        requests.push_back(packData<std::string>("SetLoadWeight", paramsWeight));

        std::vector<std::string> paramsCentroid{ "0",
            std::to_string(payload_to_tcp_[0]), std::to_string(payload_to_tcp_[1]), std::to_string(payload_to_tcp_[2]) };
        requests.push_back(packData<std::string>("SetLoadCoord", paramsCentroid));

        paramsEnable[0] = 1;
        requests.push_back(packData<int>("RobotEnable", paramsEnable));

        // delay 200ms
        requests.push_back("delay:200");

        auto feedbacks = ((DriverSocketFair*)drivers_map_[name_].get())->request(requests);
        bool res = true;
        for (const auto& it : feedbacks)
        {
            if (it.empty() || it.find("errcode") != std::string::npos)
            {
                res = false;
                break;
            }
        }

        return res;
    }

    bool FairHardwareInterface::tcp_close()
    {
        std::vector<std::string> requests;
        std::vector<int> paramsEnable{ 0 };
        requests.push_back(packData<int>("RobotEnable", paramsEnable));

        auto feedbacks = ((DriverSocketFair*)drivers_map_[name_].get())->request(requests);
        int index = -1, id = 0;
        std::vector<std::string> data;
        auto res = parseFeedback(feedbacks.front(), index, id, data);
        return res && id == CMD_MAP_.at("RobotEnable");
    }

    bool FairHardwareInterface::tcp_state()
    {
        std::vector<std::string> requests;
        requests.push_back(packData("GetSoftwareVersion"));

        std::vector<std::string> feedbacks = ((DriverSocketFair*)drivers_map_[name_].get())->request(requests);
        int index = -1, id = 0;
        std::vector<std::string> data;
        auto res = parseFeedback(feedbacks.front(), index, id, data);
        if (res && id == CMD_MAP_.at("GetSoftwareVersion"))
        {
            std::cout << "FAIR robot version: " << data.front() << std::endl;
        }

        return res;
    }

    bool FairHardwareInterface::tcp_read()
    {
        std::vector<std::string> requests;
        requests.push_back(packData("GetActualJointPosRadian"));

        std::vector<std::string> feedbacks = ((DriverSocketFair*)drivers_map_[name_].get())->request(requests);
        int index = -1, id = 0;
        std::vector<std::string> data;
        auto res = parseFeedback(feedbacks.front(), index, id, data);
        if (res && id == CMD_MAP_.at("GetActualJointPosRadian"))
        {
            for (std::size_t i = 0; i < std::min(joint_position_.size(), data.size()); ++i)
            {
                joint_position_[i] = std::stod(data[i]);
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

    bool FairHardwareInterface::tcp_servoPositions(const std::vector<double>& Positions, double Duration)
    {
        std::vector<std::string> requests;
        std::vector<double> params;
        for (const auto& it : Positions)
        {
            params.push_back(angles::to_degrees(it));
        }
        params.insert(params.end(), { 0.0, 0.0, Duration, 0.0, 0.0 });
        requests.push_back(packData<double>("ServoJ", params));

        auto feedbacks = ((DriverSocketFair*)drivers_map_[name_].get())->request(requests);
        int index = -1, id = 0;
        std::vector<std::string> data;
        auto res = parseFeedback(feedbacks.front(), index, id, data);
        return res && id == CMD_MAP_.at("ServoJ");
    }

    bool FairHardwareInterface::tcp_setIo(int Addr, int Level)
    {
        std::vector<std::string> requests;
        std::vector<int> paramsDo{ Addr, Level, 1 }; // 1: smooth
        requests.push_back(packData<int>("SetDO", paramsDo));

        auto feedbacks = ((DriverSocketFair*)drivers_map_[name_].get())->request(requests);
        int index = -1, id = 0;
        std::vector<std::string> data;
        auto res = parseFeedback(feedbacks.front(), index, id, data);
        return res && id == CMD_MAP_.at("SetDO");
    }

    bool FairHardwareInterface::tcp_isProtective()
    {
        // TODO
        return false;
    }

    bool FairHardwareInterface::tcp_protectiveRecover()
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

        auto res = ((DriverSocketFair*)drivers_map_[name_].get())->request(requests);
        if (!res.empty())
        {
            ROS_WARN_STREAM("failed to recover from protective state");
        }
        return res.empty();
    }

    bool FairHardwareInterface::api_init(const std::string& Addr)
    {
        // if (api_instance_->RobotEnable(0) != ERR_SUCCESS)
        // {
        //     ROS_ERROR_STREAM("failed to enable robot. failed to initialize FAIR driver");
        //     return false;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // if (api_instance_->Mode(0) != ERR_SUCCESS)
        // {
        //     ROS_ERROR_STREAM("failed to set auto mode. failed to initialize FAIR driver");
        //     return false;
        // }
        // if (api_instance_->SetSpeed(int(velocity_scale_ * 100)) != ERR_SUCCESS)
        // {
        //     ROS_ERROR_STREAM("failed to set velocity scale to " << velocity_scale_ << ". failed to initialize FAIR driver");
        //     return false;
        // }
        // if (api_instance_->SetLoadWeight(payload_weight_) != ERR_SUCCESS)
        // {
        //     ROS_ERROR_STREAM("failed to set payload weight. failed to initialize FAIR driver");
        //     return false;
        // }
        // whi_fair::DescTran centroid;
        // centroid.x = payload_to_tcp_[0];
        // centroid.y = payload_to_tcp_[1];
        // centroid.z = payload_to_tcp_[2];
        // if (api_instance_->SetLoadCoord(&centroid) != ERR_SUCCESS)
        // {
        //     ROS_ERROR_STREAM("failed to set payload coord. failed to initialize FAIR driver");
        //     return false;
        // }
        // if (api_instance_->RobotEnable(1) != ERR_SUCCESS)
        // {
        //     ROS_ERROR_STREAM("failed to enable robot. failed to initialize FAIR driver");
        //     return false;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // ROS_INFO_STREAM("FAIR driver is initialized successfully");
        return true;
    }

    void FairHardwareInterface::api_close()
    {
        // api_instance_->RobotEnable(0);
        // api_instance_->CloseRPC();
    }

    bool FairHardwareInterface::api_read()
    {
        whi_fair::JointPos jointPos;
//         if (api_instance_->GetActualJointPosDegree(0, &jointPos) == ERR_SUCCESS)
//         {
//             for (std::size_t i = 0; i < std::min(joint_position_.size(), sizeof(jointPos.jPos)); ++i)
//             {
//                 joint_position_[i] = angles::from_degrees(jointPos.jPos[i]);
//             }

// #ifdef DEBUG
//             std::cout << "read positions:";
//             for (const auto& it : joint_position_)
//             {
//                 std::cout << it << ",";
//             }
//             std::cout << std::endl;
// #endif
//             return true;
//         }
//         else
        {
            return false;
        }
    }

    bool FairHardwareInterface::api_servoPositions(const std::vector<double>& Positions, double Duration)
    {
        whi_fair::JointPos positions;
        for (int i = 0; i < std::min(Positions.size(), sizeof(positions.jPos)); ++i)
        {
            positions.jPos[i] = angles::to_degrees(Positions[i]);
        }
#ifdef DEBUG
        std::cout << "commanded positions:";
        for (const auto& it : positions.jPos)
        {
            std::cout << it << ",";
        }
#endif
        whi_fair::ExaxisPos extPositions;
        // auto res = api_instance_->ServoJ(&positions, &extPositions, 0, 0, Duration, 0, 0);
        // if (res != ERR_SUCCESS)
        // {
        //     ROS_WARN_STREAM("failed to execute ServoJ motion with error code: " << res);
        // }

        // return res == ERR_SUCCESS;
        return false;
    }

    bool FairHardwareInterface::api_setIo(int Addr, int Level)
    {
        // return api_instance_->SetDO(Addr, Level, 1, 0) == ERR_SUCCESS;
        return false;
    }

    bool FairHardwareInterface::api_isProtective()
    {
        bool res = false;
        // api_instance_->is_in_collision(&res);

        return res;
    }

    bool FairHardwareInterface::api_protectiveRecover()
    {
        // return api_instance_->collision_recover() == ERR_SUCC &&
        //     api_instance_->servo_move_enable(true) == ERR_SUCC;
        return false;
    }

    void FairHardwareInterface::makeOffers()
    {
        // advertise arm ready service
        server_ready_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("arm_ready", &FairHardwareInterface::onServiceReady, this));            
        // advertise io service
        server_io_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("arm_io", &FairHardwareInterface::onServiceIo, this));
        // create state publisher
        pub_motion_state_ = std::make_unique<ros::Publisher>(
            node_handle_->advertise<whi_interfaces::WhiMotionState>("arm_motion_state", 1));
    }

    bool FairHardwareInterface::onServiceReady(std_srvs::Trigger::Request& Request, std_srvs::Trigger::Response& Response)
    {
        return (Response.success = standby_);
    }

    bool FairHardwareInterface::onServiceIo(whi_interfaces::WhiSrvIo::Request& Request,
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
                if (name_ == hardware[API])
                {
                    Response.result = api_setIo(Request.addr, Request.level);
                }
                else if (name_ == hardware[SOCKET])
                {
                    Response.result = tcp_setIo(Request.addr, Request.level);
                }
            }
        }

        return Response.result;
    }
}
