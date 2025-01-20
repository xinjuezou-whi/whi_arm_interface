/******************************************************************
arm hardware interface of JAKA under ROS 1
it is a hardware resouces layer for ros_controller

Features:
- JAKA hardware interfaces
- xxx

Dependency:
- libjakaAPI.so
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-09-03: Initial version
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_arm_hardware_base.h"
#include "whi_interfaces/WhiSrvIo.h"
#include <std_srvs/Trigger.h>
#include "jakaAPI/JAKAZuRobot.h"

namespace whi_arm_hardware_interface
{
    class JakaHardwareInterface : public ArmHardware
    {
    protected:
        enum ParamKey { JOINT_POS = 0, PROTECTIVE_STOP, ENABLE, POWER, IN_SERVO, PARAM_KEY_SUM };
        static constexpr const char* paramKey[PARAM_KEY_SUM] = { "joint_pos", "protective_stop", "enable", "power", "in_servomove" };

    public:
        JakaHardwareInterface(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        virtual ~JakaHardwareInterface();

    public:
        void quit() override;

    protected:
        void init();
        void update(const ros::TimerEvent& Event);
        void read();
        void write(ros::Duration ElapsedTime);
        void initializing();
        // TCP protocol related
        bool tcp_init();
        bool tcp_close();
        bool tcp_state();
        bool tcp_read();
        bool tcp_servoPositions(const std::vector<double>& Positions, double Duration);
        bool tcp_setIo(int Addr, int Level);
        bool tcp_isProtective();
        bool tcp_protectiveRecover();
        // API related
        bool api_init(const std::string& Addr);
        void api_close();
        bool api_read();
        bool api_servoPositions(const std::vector<double>& Positions, double Duration);
        bool api_setIo(int Addr, int Level);
        bool api_isProtective();
        bool api_protectiveRecover();
        void makeOffers();
        bool onServiceReady(std_srvs::Trigger::Request& Request, std_srvs::Trigger::Response& Response);
        bool onServiceIo(whi_interfaces::WhiSrvIo::Request& Request,
            whi_interfaces::WhiSrvIo::Response& Response);

    protected:
        enum HomingState { STA_TO_HOME = 0, STA_HOMING, STA_HOMED };

    protected:
        std::string name_{ "socket" };
        std::string controller_type_{ "position" };
        std::string addr_{ "10.5.5.1" };
        std::unique_ptr<JAKAZuRobot> api_instance_{ nullptr };
        double velocity_scale_{ 1.0 };
        double payload_weight_{ 0.0 };
        std::vector<double> payload_to_tcp_;
        bool standby_{ false };
        bool is_protective_{ false };
        double lpf_{ 0.5 };
    };
}
