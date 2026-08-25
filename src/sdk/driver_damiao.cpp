#include "whi_arm_interface/driver_damiao.h"

#include <array>
#include <chrono>
#include <iostream>
#include <sstream>

DriverDamiao::DriverDamiao(const std::string& JointName, const std::string& BusAddr,
    uint16_t SendId, uint16_t RecvId, int /*DriveType*/, int /*DriveMode*/, double /*ReductionRatio*/)
    : DriverBase(JointName), bus_addr_(BusAddr), send_id_(SendId), recv_id_(RecvId)
{
    bus_ = std::make_shared<CanBus>(bus_addr_);
    // apply the kernel-side receive filter BEFORE open(), so this socket only
    // ever receives frames with can_id == recv_id_ (this joint's real
    // feedback frame). Without this, every joint's socket on a shared bus
    // sees every other joint's command loopback + response traffic too.
    bus_->setRecvFilter(recv_id_);
    if (!bus_->open())
    {
        std::cerr << "failed to open canbus " << bus_addr_ << " for joint " << joint_name_ << std::endl;
    }
}

DriverDamiao::~DriverDamiao()
{
}

void DriverDamiao::parseProtocolConfig(const std::string& ProtocolConfig)
{
    protocol_ = std::make_unique<Protocol>();
    if (!protocol_->parseProtocol(ProtocolConfig))
    {
        std::cerr << "failed to parse protocol config " << ProtocolConfig
            << " for joint " << joint_name_ << std::endl;
    }

    if (protocol_->init_commands_list_)
    {
        sendStaticCommands(*protocol_->init_commands_list_);
    }

    terminated_.store(false);
    th_read_ = std::thread(&DriverDamiao::threadReadCan, this);
}

double DriverDamiao::readAngle()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return state_position_;
}

double DriverDamiao::readVelocity()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return state_velocity_;
}

double DriverDamiao::readTorque()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return state_torque_;
}

void DriverDamiao::actuate(double Command)
{
    if (!protocol_ || !bus_ || !bus_->isOpen())
    {
        return;
    }

    OperatingParams params;
    params.params_map_["position"] = Command;
    params.params_map_["velocity"] = 0.0;
    params.params_map_["kp"] = kp_;
    params.params_map_["kd"] = kd_;
    params.params_map_["torque"] = 0.0;

    Protocol::ComposedCommandsList dataList;
    if (protocol_->composeCommand({ "mit_control" }, params, dataList) > 0)
    {
        for (const auto& pack : dataList)
        {
            bus_->write(send_id_, pack.second.size(), pack.second.data());
        }
    }
}

void DriverDamiao::actuate(std::string /*Command*/)
{
    // 达妙MIT模式无字符串指令，留空
}

void DriverDamiao::close()
{
    // NEW: idempotency guard. close() can now legitimately be called twice
    // in the lifetime of this object:
    //   1) WhiArmInterface::on_deactivate() -> ArmHardwareOpenarm::quit()
    //      -> this close(), on every lifecycle deactivate
    //   2) ~ArmHardwareOpenarm() -> quit() -> this close() again, as a
    //      destructor-time fallback in case deactivate never ran (e.g.
    //      process killed before the lifecycle transition completed)
    // exchange(true) atomically reads-the-old-value-and-sets-true; if some
    // other call already flipped it to true, we bail out immediately instead
    // of re-sending the deactivate command list and re-joining th_read_
    // (joining an already-joined std::thread is undefined behavior --
    // std::terminate on most implementations).
    if (closed_.exchange(true))
    {
        return;
    }

    if (protocol_ && protocol_->deactivate_commands_list_)
    {
        sendStaticCommands(*protocol_->deactivate_commands_list_);
    }
    terminated_.store(true);
    if (th_read_.joinable())
    {
        th_read_.join();
    }
    if (bus_)
    {
        bus_->close();
    }
}

void DriverDamiao::set_debug_params(const std::map<std::string, bool>& DebugParams)
{
    debug_params_ = DebugParams;
}

bool DriverDamiao::sendStaticCommands(const Protocol::StaticCommandsList& CommandsList)
{
    if (!bus_ || !bus_->isOpen())
    {
        return false;
    }

    for (const auto& cmd : CommandsList)
    {
        bus_->write(send_id_, cmd.data_.size(), cmd.data_.data());
        if (cmd.delay_ > 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(cmd.delay_));
        }
    }
    return true;
}

void DriverDamiao::threadReadCan()
{
    while (!terminated_.load())
    {
        if (!bus_ || !bus_->isOpen())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        unsigned int canId = 0;
        unsigned char raw[8] = { 0 };
        ssize_t n = bus_->read(canId, raw);

        // NEW: with CanBus now applying a recv timeout (SO_RCVTIMEO, see
        // canbus.cpp), a timed-out read comes back as n <= 0 just like a
        // real read error would. Either way this is the correct behavior
        // here: loop back around and re-check terminated_. Before this fix,
        // read() blocked indefinitely with no timeout, so if terminated_
        // was set (from close()) while no frame was arriving on the bus,
        // th_read_ would never wake up to observe it -- close()'s
        // th_read_.join() would then hang, which is what stalled shutdown
        // long enough to hit the 5s SIGINT->SIGTERM escalation seen in the
        // launch log.
        if (n <= 0 || canId != recv_id_ || !protocol_)
        {
            continue;
        }

        // byte0 高4位是 ERR，不参与位打包解析，单独判
        error_code_.store((raw[0] >> 4) & 0x0f);

        std::array<uint8_t, 8> rawArr;
        std::copy(std::begin(raw), std::end(raw), rawArr.begin());
        auto feedbacks = protocol_->retrieveFeedback(0x00, 0, rawArr);

        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto& fb : feedbacks)
        {
            try
            {
                if (fb.first == "position") state_position_ = std::any_cast<double>(fb.second);
                else if (fb.first == "velocity") state_velocity_ = std::any_cast<double>(fb.second);
                else if (fb.first == "torque") state_torque_ = std::any_cast<double>(fb.second);
                else if (fb.first == "mos_temp") state_mos_temp_ = int(std::any_cast<long>(fb.second));
                else if (fb.first == "rotor_temp") state_rotor_temp_ = int(std::any_cast<long>(fb.second));
            }
            catch (const std::bad_any_cast& e)
            {
                std::cerr << "bad_any_cast decoding feedback '" << fb.first
                    << "' for joint " << joint_name_ << ": " << e.what() << std::endl;
            }
        }
    }
}

void DriverDamiao::enable()
{
    // 保留接口，暂时未接入外部调用点（如需要单独使能/去使能某关节，可在这里发 static_commands 里 "set zero" 或自定义 group）
}
