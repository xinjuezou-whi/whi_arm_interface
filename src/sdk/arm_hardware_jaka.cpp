/******************************************************************
arm hardware interface of JAKA under ROS 2
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
#include "whi_arm_interface/hw_config_jaka.h"
#include <json/json.h>

#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>

#include <thread>

namespace whi_arm_hardware_interface
{
    JakaHardwareInterface::JakaHardwareInterface(const std::string& Config, rclcpp::Node::SharedPtr Node)
        : ArmHardware(Config, Node)
    {
        parseConfig(Config);
        init();
    }

    void JakaHardwareInterface::quit()
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
        // drivers
        if (hw_config_->payload_to_tcp_.empty())
        {
            hw_config_->payload_to_tcp_.resize(3);
        }
        else
        {
            // JAKA requires mm
            for (auto& it : hw_config_->payload_to_tcp_)
            {
                it *= 1000.0;
            }
        }

        initializing();
    }

    bool JakaHardwareInterface::setIo(int Addr, int Level)
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

    void JakaHardwareInterface::read(WhiArmInterface* HwIf, double Dt)
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

    void JakaHardwareInterface::write(WhiArmInterface* HwIf, double Dt)
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

    void JakaHardwareInterface::initializing()
    {
        auto addr = hw_config_->hardware_ == hardware[API] ? hw_config_->api_addr_ : hw_config_->socket_addr_;
        while (!ping(addr))
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to ping:" << addr << ", attempt to another try in " << hw_config_->startup_duration_ << " seconds");
            std::this_thread::sleep_for(std::chrono::duration<double>(hw_config_->startup_duration_));
        }
        
        bool res = false;
        while (!res)
        {
            if (hw_config_->hardware_ == hardware[API])
            {
                if (!api_instance_)
                {
                    api_instance_ = std::make_unique<JAKAZuRobot>();
                }
                res = (api_instance_->login_in(addr.c_str()) == ERR_SUCC);
            }
            else if (hw_config_->hardware_ == hardware[SOCKET])
            {
                drivers_map_[hw_config_->hardware_] = std::make_unique<DriverSocketJson>(hw_config_->hardware_, addr, 10001);
                drivers_map_[hw_config_->hardware_]->set_debug_params(
                    std::map<std::string, bool>{{ "print_tcp_feedback", hw_config_->debug_print_tcp_feedback_ }});
                ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->setParamsKey(paramKey, PARAM_KEY_SUM);
                res = tcp_state();
                if (!res)
                {
                    tcp_close();
                    drivers_map_[hw_config_->hardware_]->close();
                }
            }

            if (!res)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to setup connection, attempt to another try in " << hw_config_->startup_duration_ << " seconds");
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
                RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to initialize, attempt to another try in " << hw_config_->startup_duration_ << " seconds");
                std::this_thread::sleep_for(std::chrono::duration<double>(hw_config_->startup_duration_));
            }
        }
    }

    bool JakaHardwareInterface::tcp_init()
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
        root["lpf_cf"] = hw_config_->lpf_;
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"rapid_rate","rate_value":1.0}
        root["cmdName"] = "rapid_rate";
        root["rate_value"] = hw_config_->velocity_scale_;
        requests.push_back(Json::writeString(builder, root));
        // {"cmdName":"set_tool_payload","mass":weight,"centroid":[x,y,z]}
        root["cmdName"] = "set_tool_payload";
        root["mass"] = hw_config_->payload_weight_;
        for (const auto& it : hw_config_->payload_to_tcp_)
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

        return ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->request(requests).empty();
    }

    bool JakaHardwareInterface::tcp_close()
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

        return ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->request(requests).empty();
    }

    bool JakaHardwareInterface::tcp_state()
    {
        Json::Value root;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"get_robot_state"}
        root["cmdName"] = "get_robot_state";
        requests.push_back(Json::writeString(builder, root));

        return ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->request(requests).empty();
    }

    bool JakaHardwareInterface::tcp_read()
    {
        Json::Value root;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"get_joint_pos"}
        root["cmdName"] = "get_joint_pos";
        requests.push_back(Json::writeString(builder, root));

        ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->request(requests);
        auto read = ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->readParam(paramKey[JOINT_POS]);
        if (!read.empty())
        {
            for (std::size_t i = 0; i < std::min(joint_positions_.size(), read.size()); ++i)
            {
                joint_positions_[i] = angles::from_degrees(read[i]);
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

    bool JakaHardwareInterface::tcp_servoPositions(const std::vector<double>& Positions, double Duration)
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

        auto res = ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->request(requests);
        if (!res.empty())
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to execute servo_j motion");
        }
        return res.empty();
    }

    bool JakaHardwareInterface::tcp_setIo(int Addr, int Level)
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

        auto res = ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->request(requests);
        if (!res.empty())
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to execute set digital output");
        }
        return res.empty();
    }

    bool JakaHardwareInterface::tcp_isProtective()
    {
        Json::Value root;
        Json::Value data;
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";

        std::vector<std::string> requests;
        // {"cmdName":"protective_stop_status"}
        root["cmdName"] = "protective_stop_status";
        requests.push_back(Json::writeString(builder, root));

        if (((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->request(requests).empty())
        {
            auto read = ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->readParamStr(paramKey[PROTECTIVE_STOP]);
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
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to requiry protective state");
            return false;
        }
    }

    bool JakaHardwareInterface::tcp_protectiveRecover()
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

        auto res = ((DriverSocketJson*)drivers_map_[hw_config_->hardware_].get())->request(requests);
        if (!res.empty())
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to recover from protective state");
        }
        return res.empty();
    }

    bool JakaHardwareInterface::api_init(const std::string& Addr)
    {
        bool res = true;

        if (api_instance_->login_in(Addr.c_str()) != ERR_SUCC)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to login JAKA driver. failed to initialize JAKA driver"
                << "\033[0m");
            api_instance_ = nullptr;
            return false;
        }
        if (api_instance_->servo_move_enable(false) != ERR_SUCC)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to disable servo mode. failed to initialize JAKA driver"
                << "\033[0m");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (api_instance_->servo_move_use_joint_LPF(hw_config_->lpf_) != ERR_SUCC)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to set LPF to " << hw_config_->lpf_ << " Hz. failed to initialize JAKA driver"
                << "\033[0m");
            return false;
        }
        if (api_instance_->set_rapidrate(hw_config_->velocity_scale_) != ERR_SUCC)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to set velocity scale to " << hw_config_->velocity_scale_ << ". failed to initialize JAKA driver"
                << "\033[0m");
            return false;
        }
        PayLoad payload;
        payload.mass = hw_config_->payload_weight_;
        payload.centroid.x = hw_config_->payload_to_tcp_[0];
        payload.centroid.y = hw_config_->payload_to_tcp_[1];
        payload.centroid.z = hw_config_->payload_to_tcp_[2];
        if (api_instance_->set_payload(&payload) != ERR_SUCC)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to set payload. failed to initialize JAKA driver"
                << "\033[0m");
            return false;
        }
        if (api_instance_->power_on() != ERR_SUCC)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to power on arm. failed to initialize JAKA driver"
                << "\033[0m");
            return false;
        }
        if (api_instance_->enable_robot() != ERR_SUCC)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to enable arm. failed to initialize JAKA driver"
                << "\033[0m");
            return false;
        }
        if (api_instance_->servo_move_enable(true) != ERR_SUCC)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("WhiArmInterface"), "\033[1;31m" <<
                "failed to enable servo mode. failed to initialize JAKA driver"
                << "\033[0m");
            return false;
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("WhiArmInterface"), "JAKA driver is initialized successfully");
        return true;
    }

    void JakaHardwareInterface::api_close()
    {
        api_instance_->servo_move_enable(false);
        api_instance_->disable_robot();
        api_instance_->power_off();
        api_instance_->login_out();
    }

    bool JakaHardwareInterface::api_read()
    {
        JointValue jointPos;
        if (api_instance_->get_joint_position(&jointPos) == ERR_SUCC)
        {
            for (std::size_t i = 0; i < std::min(joint_positions_.size(), sizeof(jointPos.jVal)); ++i)
            {
                joint_positions_[i] = jointPos.jVal[i];
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

    bool JakaHardwareInterface::api_servoPositions(const std::vector<double>& Positions, double Duration)
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

        auto res = api_instance_->servo_j(&positions, MoveMode::ABS, stepNum);
        if (res != ERR_SUCC)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("WhiArmInterface"), "failed to execute servo_j motion with error code: " << res);
        }

        return res == ERR_SUCC;
    }

    bool JakaHardwareInterface::api_setIo(int Addr, int Level)
    {
        return api_instance_->set_digital_output(IO_CABINET, Addr - 1, Level) == ERR_SUCC;
    }

    bool JakaHardwareInterface::api_isProtective()
    {
        BOOL res = FALSE;
        api_instance_->is_in_collision(&res);

        return res;
    }

    bool JakaHardwareInterface::api_protectiveRecover()
    {
        return api_instance_->collision_recover() == ERR_SUCC &&
            api_instance_->servo_move_enable(true) == ERR_SUCC;
    }
}
