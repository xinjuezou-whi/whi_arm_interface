/******************************************************************
达妙(Damiao)电机 MIT 模式 CAN 驱动，基于通用 Protocol 位打包引擎

Written by Xinjue Zou
Apache License Version 2.0

Changelog:
2026-08-12: adapt to whi_arm_interface::DriverBase interface
2026-08-21: make close() idempotent (closed_ is now atomic_bool) --
            close() can now be reached both from
            WhiArmInterface::on_deactivate() (via hardware_->quit()) AND
            later from ~ArmHardwareOpenarm() as a destructor-time
            fallback; without the guard the second call would re-send
            the deactivate command list and re-join an already-joined
            th_read_, the latter being undefined behavior
            (std::terminate on most implementations)
******************************************************************/
#pragma once
#include "driver_base.h"
#include "canbus.h"
#include "protocol_def.h"

#include <thread>
#include <mutex>
#include <atomic>

class DriverDamiao : public DriverBase
{
public:
    DriverDamiao() = delete;
    DriverDamiao(const std::string& JointName, const std::string& BusAddr,
        uint16_t SendId, uint16_t RecvId, int DriveType, int DriveMode, double ReductionRatio);
    ~DriverDamiao() override;

public:
    // DriverBase overrides
    double readAngle() override;
    void actuate(double Command) override;
    void actuate(std::string Command) override;
    void close() override;
    std::shared_ptr<RotaryEncoderBase> getEncoder() override { return nullptr; }; // 达妙自带编码器，不走独立RotaryEncoder体系
    void cal_angularVel2PwmDuty() override {}; // 达妙走位/速/力控制，不需要 PWM 占空比换算
    void set_debug_params(const std::map<std::string, bool>& DebugParams) override;

public:
    // specific，非DriverBase虚函数
    void parseProtocolConfig(const std::string& ProtocolConfig);
    void setGains(double Kp, double Kd) { kp_ = Kp; kd_ = Kd; };
    void enable();
    double readVelocity();
    double readTorque();
    int errorCode() const { return error_code_.load(); };

protected:
    bool sendStaticCommands(const Protocol::StaticCommandsList& CommandsList);
    void threadReadCan();

protected:
    std::string bus_addr_;
    std::shared_ptr<CanBus> bus_{ nullptr };
    std::unique_ptr<Protocol> protocol_{ nullptr };

    uint16_t send_id_{ 0 };  // ESC_ID，控制帧目标ID
    uint16_t recv_id_{ 0 };  // MST_ID，反馈帧匹配ID

    double kp_{ 50.0 };
    double kd_{ 1.0 };

    std::mutex mtx_;
    double state_position_{ 0.0 };
    double state_velocity_{ 0.0 };
    double state_torque_{ 0.0 };
    int state_mos_temp_{ 0 };
    int state_rotor_temp_{ 0 };
    std::atomic_int error_code_{ 0 };
    std::map<std::string, bool> debug_params_;

    std::thread th_read_;
    std::atomic_bool terminated_{ false };
    // NEW: guards close() so it only actually runs once, no matter how many
    // times it's invoked (on_deactivate -> hardware_->quit() -> close(),
    // and/or destructor -> quit() -> close()). Using an atomic + exchange
    // (rather than a plain bool) makes the check-and-set itself thread-safe.
    std::atomic_bool closed_{ false };
};