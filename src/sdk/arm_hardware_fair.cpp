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
#include "whi_arm_interface/hw_config_fair.h"
#include <json/json.h>

#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>

#include <thread>
#include <regex>

namespace whi_arm_interface
{
    FairHardwareInterface::FairHardwareInterface(const std::string& Config, rclcpp::Node::SharedPtr Node)
        : ArmHardware(Config, Node)
    {
        parseConfig(Config);
        init();
    }

    void FairHardwareInterface::quit()
    {
        // give time to thirdparty dependencies
        std::this_thread::sleep_for(std::chrono::milliseconds(hw_config_->shutdown_patience_));

        if (hw_config_->hardware_ == hardware[API])
        {
            api_close();
        }
        else if (hw_config_->hardware_ == hardware[SOCKET])
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
        // drivers
        if (hw_config_->payload_to_tcp_.empty())
        {
            hw_config_->payload_to_tcp_.resize(3);
        }
        else
        {
            // FAIR requires mm
            for (auto& it : hw_config_->payload_to_tcp_)
            {
                it *= 1000.0; 
            }
        }

        initializing();
    }

    std::string FairHardwareInterface::getSwEstopTopic() const
    {
        return hw_config_ ? hw_config_->sw_estop_topic_ : "na";
    }

    bool FairHardwareInterface::setIo(int Addr, int Level)
    {
        if (Addr < 1 || Addr > 7)
        {
            return false;
        }

        bool res = true;
        if (hw_config_->hardware_ == hardware[API])
        {
            res = api_setIo(Addr, Level);
        }
        else if (hw_config_->hardware_ == hardware[SOCKET])
        {
            res = tcp_setIo(Addr, Level);
        }

        return res;
    }

    bool FairHardwareInterface::parseConfig(const std::string& Config)
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
                hw_config_->velocity_scale_ = root["velocity_scale"].as<double>();
                hw_config_->payload_weight_ = root["payload_weight"].as<double>();
                hw_config_->payload_to_tcp_ = root["payload_to_tcp"].as<std::vector<double>>();
                hw_config_->hardware_ = root["hardware"].as<std::string>();
                hw_config_->startup_duration_ = root["startup_duration"].as<double>();

                const auto& socket = root["socket"];
                if (socket)
                {
                    hw_config_->socket_addr_ = socket["addr"].as<std::string>();
                    hw_config_->socket_port_ = socket["port"].as<int>();
                }
                const auto& api = root["api"];
                if (api)
                {
                    hw_config_->api_addr_ = api["addr"].as<std::string>();
                }

                const auto& debug = root["debug"];
                if (debug)
                {
                    hw_config_->debug_print_tcp_feedback_ = debug["print_tcp_feedback"].as<bool>();
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

    void FairHardwareInterface::read(WhiArmInterface* HwIf, double Dt)
    {
        if (initialized_)
        {
            static bool first = true;

            bool res = true;
            if (hw_config_->hardware_ == hardware[API])
            {
                res = api_read();
                // only get_robot_status is multi-thread safe, therefore taking synchronous mech
                is_protective_ = api_isProtective();
            }
            else if (hw_config_->hardware_ == hardware[SOCKET])
            {
                res = tcp_read();
                is_protective_ = tcp_isProtective();
            }

            if (first && res)
            {
                joint_position_commands_ = joint_positions_;
                first = false;
                standby_ = true;
            }

            if (is_protective_)
            {
                trajectory_action_client_->async_cancel_all_goals();

                publishState(whi_interfaces::msg::WhiState::WARN, "state", "protective stopped");
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                    "arm entered protective state"
                    << "\033[0m");

                if (hw_config_->hardware_ == hardware[API])
                {
                    api_protectiveRecover();
                }
                else if (hw_config_->hardware_ == hardware[SOCKET])
                {
                    tcp_protectiveRecover();
                }
            }
            else
            {
                publishState(whi_interfaces::msg::WhiState::INFO, "state", "standby");
            }
        }
    }

    void FairHardwareInterface::write(WhiArmInterface* HwIf, double Dt)
    {
        if (standby_)
        {
            if (hw_config_->hardware_ == hardware[API])
            {
                api_servoPositions(joint_position_commands_, Dt);
            }
            else if (hw_config_->hardware_ == hardware[SOCKET])
            {
                tcp_servoPositions(joint_position_commands_, Dt);
            }
        }
    }

    void FairHardwareInterface::initializing()
    {
        auto addr = hw_config_->hardware_ == hardware[API] ? hw_config_->api_addr_ : hw_config_->socket_addr_;
        while (!ping(addr))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"),
                "failed to ping:" << addr << ", attempt to another try in " << hw_config_->startup_duration_ << " seconds");
            std::this_thread::sleep_for(std::chrono::duration<double>(hw_config_->startup_duration_));
        }
        
        bool res = false;
        while (!res)
        {
            if (hw_config_->hardware_ == hardware[API])
            {
                // if (!api_instance_)
                // {
                //     api_instance_ = std::make_unique<whi_fair::FRRobot>();
                // }
                // res = (api_instance_->RPC(addr.c_str()) == ERR_SUCCESS);
                // char version[64] = {0};
                // api_instance_->GetSDKVersion(version);
                // if (!res)
                // {
                //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to instance FAIR SDK, please check the version: " << version);
                // }
            }
            else if (hw_config_->hardware_ == hardware[SOCKET])
            {
                drivers_map_[hw_config_->hardware_] = std::make_unique<DriverSocketFair>(hw_config_->hardware_, addr, 8080);
                drivers_map_[hw_config_->hardware_]->set_debug_params(
                    std::map<std::string, bool>{{ "print_tcp_feedback", hw_config_->debug_print_tcp_feedback_ }});

                res = tcp_state();
                if (!res)
                {
                    tcp_close();
                    drivers_map_[hw_config_->hardware_]->close();
                }
            }

            if (!res)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"),
                    "failed to setup connection, attempt to another try in " << hw_config_->startup_duration_ << " seconds");
                std::this_thread::sleep_for(std::chrono::duration<double>(hw_config_->startup_duration_));
            }
        }

        while (!initialized_)
        {
            if (hw_config_->hardware_ == hardware[API])
            {
                initialized_ = api_init(addr);
            }
            else if (hw_config_->hardware_ == hardware[SOCKET])
            {
                initialized_ = tcp_init();
            }

            if (!initialized_)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"),
                    "failed to initialize, attempt to another try in " << hw_config_->startup_duration_ << " seconds");
                std::this_thread::sleep_for(std::chrono::duration<double>(hw_config_->startup_duration_));
            }
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

        std::vector<int> paramsSpeed{ int(hw_config_->velocity_scale_ * 100.0) };
        requests.push_back(packData<int>("SetSpeed", paramsSpeed));

        std::vector<std::string> paramsWeight{ "0", std::to_string(hw_config_->payload_weight_) };
        requests.push_back(packData<std::string>("SetLoadWeight", paramsWeight));

        std::vector<std::string> paramsCentroid{ "0",
            std::to_string(hw_config_->payload_to_tcp_[0]), std::to_string(hw_config_->payload_to_tcp_[1]), std::to_string(hw_config_->payload_to_tcp_[2]) };
        requests.push_back(packData<std::string>("SetLoadCoord", paramsCentroid));

        paramsEnable[0] = 1;
        requests.push_back(packData<int>("RobotEnable", paramsEnable));

        // delay 200ms
        requests.push_back("delay:200");

        auto feedbacks = ((DriverSocketFair*)drivers_map_[hw_config_->hardware_].get())->request(requests);
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

        auto feedbacks = ((DriverSocketFair*)drivers_map_[hw_config_->hardware_].get())->request(requests);
        int index = -1, id = 0;
        std::vector<std::string> data;
        auto res = parseFeedback(feedbacks.front(), index, id, data);
        return res && id == CMD_MAP_.at("RobotEnable");
    }

    bool FairHardwareInterface::tcp_state()
    {
        std::vector<std::string> requests;
        requests.push_back(packData("GetSoftwareVersion"));

        std::vector<std::string> feedbacks = ((DriverSocketFair*)drivers_map_[hw_config_->hardware_].get())->request(requests);
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

        std::vector<std::string> feedbacks = ((DriverSocketFair*)drivers_map_[hw_config_->hardware_].get())->request(requests);
        int index = -1, id = 0;
        std::vector<std::string> data;
        auto res = parseFeedback(feedbacks.front(), index, id, data);
        if (res && id == CMD_MAP_.at("GetActualJointPosRadian"))
        {
            for (std::size_t i = 0; i < std::min(joint_positions_.size(), data.size()); ++i)
            {
                joint_positions_[i] = std::stod(data[i]);
            }

#ifdef DEBUG
            std::cout << "read positions:";
            for (const auto& it : joint_positions_)
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

        auto feedbacks = ((DriverSocketFair*)drivers_map_[hw_config_->hardware_].get())->request(requests);
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

        auto feedbacks = ((DriverSocketFair*)drivers_map_[hw_config_->hardware_].get())->request(requests);
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

        auto res = ((DriverSocketFair*)drivers_map_[hw_config_->hardware_].get())->request(requests);
        if (!res.empty())
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to recover from protective state");
        }
        return res.empty();
    }

    bool FairHardwareInterface::api_init(const std::string& Addr)
    {
        // if (api_instance_->RobotEnable(0) != ERR_SUCCESS)
        // {
        //    RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"),
        //        "failed to enable robot. failed to initialize FAIR driver");
        //     return false;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // if (api_instance_->Mode(0) != ERR_SUCCESS)
        // {
        //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"),
        //         "failed to set auto mode. failed to initialize FAIR driver");
        //     return false;
        // }
        // if (api_instance_->SetSpeed(int(hw_config_->velocity_scale_ * 100)) != ERR_SUCCESS)
        // {
        //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"),
        //         "failed to set velocity scale to " << hw_config_->velocity_scale_ << ". failed to initialize FAIR driver");
        //     return false;
        // }
        // if (api_instance_->SetLoadWeight(hw_config_->payload_weight_) != ERR_SUCCESS)
        // {
        //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"),
        //         "failed to set payload weight. failed to initialize FAIR driver");
        //     return false;
        // }
        // whi_fair::DescTran centroid;
        // centroid.x = hw_config_->payload_to_tcp_[0];
        // centroid.y = hw_config_->payload_to_tcp_[1];
        // centroid.z = hw_config_->payload_to_tcp_[2];
        // if (api_instance_->SetLoadCoord(&centroid) != ERR_SUCCESS)
        // {
        //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"),
        //         "failed to set payload coord. failed to initialize FAIR driver");
        //     return false;
        // }
        // if (api_instance_->RobotEnable(1) != ERR_SUCCESS)
        // {
        //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"),
        //         "failed to enable robot. failed to initialize FAIR driver");
        //     return false;
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // RCLCPP_INFO_STREAM(rclcpp::get_logger("WhiArmInterface"), "FAIR driver is initialized successfully");
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
//             for (std::size_t i = 0; i < std::min(joint_positions_.size(), sizeof(jointPos.jPos)); ++i)
//             {
//                 joint_positions_[i] = angles::from_degrees(jointPos.jPos[i]);
//             }

// #ifdef DEBUG
//             std::cout << "read positions:";
//             for (const auto& it : joint_positions_)
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
        //     RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to execute ServoJ motion with error code: " << res);
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
} // namespace whi_arm_interface
