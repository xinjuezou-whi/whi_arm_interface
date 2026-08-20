#pragma once
#include <string>
#include <iostream>
#include <map>

namespace whi_arm_interface
{
    class HwConfig
    {
    public:
        class Motor
        {
        public:
            Motor() = default;
            ~Motor() = default;

        public:
            std::string protocol_config_;
            std::string bus_addr_{ "can0" };
            int device_addr_{ 0 };       // ESC_ID，控制帧目标CAN ID
            int recv_device_addr_{ 0 };  // MST_ID，反馈帧CAN ID
            double mit_kp_{ 50.0 };
            double mit_kd_{ 1.0 };
            std::pair<std::string, uint8_t> drive_type_{ "position", 1 };
            int forward_dir_{ 1 };
            int resolution_{ 16384 };
            bool multiple_{ true };
        };

    public:
        HwConfig() = default;
        ~HwConfig() = default;

    public:
        void printOut()
        {
            if (debug_print_config_)
            {
                std::cout << "******************** HwConfig" << std::endl;
                std::cout << "sw_estop_topic_: " << sw_estop_topic_ << std::endl;
                for (const auto& [name, motor] : motors_map_)
                {
                    std::cout << "  name_: " << name << std::endl;
                    std::cout << "    protocol_config_: " << motor.protocol_config_ << std::endl;
                    std::cout << "    bus_addr_: " << motor.bus_addr_ << std::endl;
                    std::cout << "    device_addr_: " << motor.device_addr_ << std::endl;
                    std::cout << "    recv_device_addr_: " << motor.recv_device_addr_ << std::endl;
                    std::cout << "    mit_kp_: " << motor.mit_kp_ << std::endl;
                    std::cout << "    mit_kd_: " << motor.mit_kd_ << std::endl;
                    std::cout << "    forward_dir_: " << motor.forward_dir_ << std::endl;
                    std::cout << "    resolution_: " << motor.resolution_ << std::endl;
                    std::cout << "    multiple_: " << motor.multiple_ << std::endl;
                }
                std::cout << "******************** end of HwConfig" << std::endl;
            }
        }

    public:
        std::string sw_estop_topic_{ "estop" };
        std::map<std::string, Motor> motors_map_;
        bool debug_print_config_{ false };
    };
} // namespace whi_arm_interface