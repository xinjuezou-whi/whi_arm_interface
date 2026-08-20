/******************************************************************
general definition for CANBUS protocol

Features:
- general definition
- bitpack encoding/decoding for cross-byte bit-packed protocols (e.g. Damiao MIT mode)
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-12-29: Initial version
2023-01-08: extended with kinds of commands
2023-01-09: multiple feedback params support
2023-06-23: group support for static commands
2023-06-24: multiple control commands support
2023-09-05: adapt variable length of control command
2026-08-10: add bitpack encoding/decoding support for cross-byte protocols
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include "printf_color.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <any>
#include <typeindex>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct OperatingParams
{
    OperatingParams() = default;
    std::map<std::string, double> params_map_;
};

class Protocol
{
public:
    const char* VERSION = "WHI generic protocol handler 00.23.0";

public:
    struct Condition
    {
        Condition() = default;
        int8_t target_{ 0 };
        std::shared_ptr<uint8_t> less_{ nullptr };
        std::shared_ptr<uint8_t> equal_{ nullptr };
        std::shared_ptr<uint8_t> great_{ nullptr };
    };

    struct Shift
    {
        Shift() = default;
        enum { SHIFT_LEFT = 0, SHIFT_RIGHT };
        uint8_t bits_{ 0 };
        uint8_t dir_{ SHIFT_LEFT };
    };

    class StaticCommand
    {
    public:
        StaticCommand() = default;
        ~StaticCommand() = default;

    public:
        std::string name_;
        std::vector<uint8_t> data_;
        uint32_t base_addr_{ 0 };
        bool is_absolute_base_{ false };
        std::map<std::string, uint8_t> variables_map_;
        uint32_t delay_{ 0 };
    };

    class CommandByte
    {
    public:
        CommandByte() = default;
        ~CommandByte() = default;

    public:
        std::any source_;
        std::shared_ptr<Condition> cond_{ nullptr };
        std::shared_ptr<Shift> shift_{ nullptr };
        bool is_absolute_{ false };
        enum { int_data = 0, float_data };
        int data_type_{ int_data };
        double multiplier_{ 1.0 };
        double divider_{ 1.0 };
        double additioner_{ 0.0 };
    };

    // NEW: a single bit-packed input field (e.g. position/velocity/kp/kd/torque in a MIT frame)
    struct BitFieldIn
    {
        BitFieldIn() = default;
        std::string source_;      // key into OperatingParams::params_map_
        uint8_t bit_width_{ 0 };
        double min_{ 0.0 };
        double max_{ 0.0 };
    };

    class ControlCommand
    {
    public:
        ControlCommand() = default;
        ~ControlCommand() = default;

    public:
        std::string name_;
        std::string family_{ std::string() };
        uint32_t base_addr_{ 0 };
        bool is_absolute_base_{ false };
        std::vector<CommandByte> data_;
        double multiplier_{ 1.0 };
        double divider_{ 1.0 };
        double additioner_{ 0.0 };
        // NEW: bitpack encoding support
        bool is_bitpack_{ false };
        uint8_t start_byte_{ 0 };
        std::vector<BitFieldIn> bit_fields_;
    };

    class FeebackByte
    {
    public:
        FeebackByte() = default;
        ~FeebackByte() = default;

    public:
        uint8_t index_{ 0 };
        Shift shift_;
    };

    // NEW: a single bit-packed output field (e.g. position/velocity/torque in a state feedback frame)
    struct BitFieldOut
    {
        BitFieldOut() = default;
        std::string name_;        // key exposed in the feedback result
        uint8_t bit_width_{ 0 };
        double min_{ 0.0 };
        double max_{ 0.0 };
    };

    class FeedbackParam
    {
    public:
        FeedbackParam() = default;
        ~FeedbackParam() = default;

    public:
        uint32_t base_addr_{ 0 };
        bool is_absolute_base_{ false };
        std::vector<FeebackByte> bytes_;
        double multiplier_{ 1.0 };
        double divider_{ 1.0 };
        double additioner_{ 0.0 };
        enum { int_data = 0, float_data };
        int data_type_{ int_data };
        Shift device_id_shift_;
        std::shared_ptr<int> extended_id_{ nullptr };
        // NEW: bitpack decoding support
        bool is_bitpack_{ false };
        uint8_t start_byte_{ 0 };
        std::vector<BitFieldOut> bit_fields_out_;
    };

    enum Type { FRAME_DATA = 0, FRAME_REMOTE, FRAME_NONE };

public:
	Protocol() = default;
	~Protocol() = default;

public:
    using StaticCommandsList = std::vector<StaticCommand>;
    using StaticCommandsMap = std::unordered_map<std::string, StaticCommand>;
    using StaticCommandsGroupsMap = std::unordered_map<std::string, StaticCommandsList>;
    using ControlCommandsMap = std::unordered_map<std::string, ControlCommand>;
    using FeedbacksMap = std::unordered_map<std::string, FeedbackParam>;
    using ComposedCommandsList = std::vector<std::pair<uint32_t, std::vector<uint8_t>>>;

public:
    std::unique_ptr<StaticCommandsList> init_commands_list_{ nullptr };
    std::unique_ptr<StaticCommandsList> deactivate_commands_list_{ nullptr };
    std::unique_ptr<StaticCommandsMap> static_commands_map_{ nullptr };
    std::unique_ptr<StaticCommandsGroupsMap> static_commands_groups_map_{ nullptr };
    std::unique_ptr<ControlCommandsMap> control_commands_map_{ nullptr };
    std::unique_ptr<FeedbacksMap> feedbacks_map_{ nullptr };

public:
    bool parseProtocol(const std::string& ProtocolConfig)
    {
        try
        {
            YAML::Node node = YAML::LoadFile(ProtocolConfig);

            // init commands
            loadStaticCommandsList(node, "init_commands", init_commands_list_);
            // deativate commands
            loadStaticCommandsList(node, "deactivate_commands", deactivate_commands_list_);
            // static commands
            loadStaticCommands(node, "static_commands", static_commands_groups_map_, static_commands_map_);
            // control commands
            loadControlCommandsMap(node, "control_commands", control_commands_map_);
            // feedbacks
            loadFeedbacksMap(node, "feedbacks", feedbacks_map_);

            return true;
        }
        catch (const std::exception& e)
        {
            printf((std::string(RED) + "[fatal] failed to load protocol config file %s" + CLEANUP + "\n").c_str(),
                ProtocolConfig.c_str());
            return false;
        }
    }

    int composeCommand(const std::vector<std::string>& Families, const OperatingParams& Params, ComposedCommandsList& DataList)
    {
        DataList.clear();

        if (control_commands_map_)
        {
            for (const auto& it : *control_commands_map_)
            {
                if (!Families.empty() &&
                    std::find(Families.begin(), Families.end(), it.second.name_) == Families.end() &&
                    std::find(Families.begin(), Families.end(), it.second.family_) == Families.end())
                {
                    continue;
                }

                std::pair<uint32_t, std::vector<uint8_t>> dataPack;
                dataPack.first = it.second.base_addr_;

                // NEW: bitpack encoding branch
                if (it.second.is_bitpack_)
                {
                    uint32_t totalBits = 0;
                    for (const auto& f : it.second.bit_fields_)
                    {
                        totalBits += f.bit_width_;
                    }
                    uint32_t numBytes = (totalBits + 7) / 8;
                    dataPack.second.assign(size_t(it.second.start_byte_) + numBytes, 0);

                    uint64_t acc = 0;
                    for (const auto& f : it.second.bit_fields_)
                    {
                        double val = 0.0;
                        if (auto search = Params.params_map_.find(f.source_); search != Params.params_map_.end())
                        {
                            val = search->second;
                        }
                        val = std::max(f.min_, std::min(val, f.max_));
                        double span = f.max_ - f.min_;
                        double norm = span != 0.0 ? (val - f.min_) / span : 0.0;
                        uint32_t raw = uint32_t(norm * double((1u << f.bit_width_) - 1) + 0.5);
                        acc = (acc << f.bit_width_) | uint64_t(raw);
                    }
                    for (uint32_t i = 0; i < numBytes; ++i)
                    {
                        dataPack.second[it.second.start_byte_ + (numBytes - 1 - i)] = uint8_t(acc >> (8 * i));
                    }

                    DataList.push_back(dataPack);
                    continue;
                }

                dataPack.second.resize(it.second.data_.size(), 0);

                for (size_t i = 0; i < it.second.data_.size(); ++i)
                {
                    try
                    {
                        dataPack.second[i] = std::any_cast<uint8_t>(it.second.data_[i].source_);
                    }
                    catch (const std::bad_any_cast& e)
                    {
                        if (std::type_index(it.second.data_[i].source_.type()) !=
                            std::type_index(typeid(void)))
                        {
                            std::string obj = std::any_cast<std::string>(it.second.data_[i].source_);
                            if (it.second.data_[i].cond_)
                            {
                                if (auto searchObj = Params.params_map_.find(obj); searchObj != Params.params_map_.end())
                                {
                                    if (searchObj->second < it.second.data_[i].cond_->target_)
                                    {
                                        if (it.second.data_[i].cond_->less_)
                                        {
                                            dataPack.second[i] = *it.second.data_[i].cond_->less_;
                                        }
                                    }
                                    else if (searchObj->second > it.second.data_[i].cond_->target_)
                                    {
                                        if (it.second.data_[i].cond_->great_)
                                        {
                                            dataPack.second[i] = *it.second.data_[i].cond_->great_;
                                        }
                                    }
                                    else
                                    {
                                        if (it.second.data_[i].cond_->equal_)
                                        {
                                            dataPack.second[i] = *it.second.data_[i].cond_->equal_;
                                        }
                                    }
                                }
                            }
                            else if (it.second.data_[i].shift_)
                            {
                                if (auto searchObj = Params.params_map_.find(obj); searchObj != Params.params_map_.end())
                                {
                                    if (it.second.data_[i].data_type_ == CommandByte::int_data)
                                    {
                                        double raw = searchObj->second * it.second.multiplier_ /
                                            it.second.divider_ + it.second.additioner_;
                                        raw = raw * it.second.data_[i].multiplier_ / it.second.data_[i].divider_ +
                                            it.second.data_[i].additioner_;
                                        long value = it.second.data_[i].is_absolute_ ? abs(long(raw)) : long(raw);
                                        dataPack.second[i] = it.second.data_[i].shift_->dir_ == Shift::SHIFT_LEFT ?
                                            int8_t(value << it.second.data_[i].shift_->bits_) :
                                            int8_t(value >> it.second.data_[i].shift_->bits_);
                                    }
                                    else if (it.second.data_[i].data_type_ == CommandByte::float_data)
                                    {
                                        float raw = searchObj->second * it.second.multiplier_ /
                                            it.second.divider_ + it.second.additioner_;
                                        raw = raw * it.second.data_[i].multiplier_ / it.second.data_[i].divider_ +
                                            it.second.data_[i].additioner_;
                                        raw = it.second.data_[i].is_absolute_ ?
                                            float(fabs(raw)) : raw;
                                        unsigned long value = *(unsigned long*)&raw;
                                        dataPack.second[i] = it.second.data_[i].shift_->dir_ == Shift::SHIFT_LEFT ?
                                            int8_t(value << it.second.data_[i].shift_->bits_) :
                                            int8_t(value >> it.second.data_[i].shift_->bits_);
                                    }
                                }
                            }
                            else
                            {
                                if (auto searchObj = Params.params_map_.find(obj); searchObj != Params.params_map_.end())
                                {
                                    if (it.second.data_[i].data_type_ == CommandByte::int_data)
                                    {
                                        double raw = searchObj->second * it.second.multiplier_ / it.second.divider_ +
                                            it.second.additioner_;
                                        raw = raw * it.second.data_[i].multiplier_ / it.second.data_[i].divider_ +
                                            it.second.data_[i].additioner_;
                                        long value = it.second.data_[i].is_absolute_ ? abs(long(raw)) : long(raw);
                                        dataPack.second[i] = uint8_t(value);
                                    }
                                    else if (it.second.data_[i].data_type_ == CommandByte::float_data)
                                    {
                                        float raw = searchObj->second * it.second.multiplier_ / it.second.divider_ +
                                            it.second.additioner_;
                                        raw = raw * it.second.data_[i].multiplier_ / it.second.data_[i].divider_ +
                                            it.second.data_[i].additioner_;
                                        raw = it.second.data_[i].is_absolute_ ? float(fabs(raw)) : raw;
                                        unsigned long value = *(unsigned long*)&raw;
                                        dataPack.second[i] = uint8_t(value);
                                    }
                                }
                                else
                                {
                                    // it is a RTR frame
                                    dataPack.second[i] = uint8_t('R');
                                }
                            }
                        }
                        else
                        {
                            dataPack.second[i] = uint8_t(0);
                        }
                    }
                }

                DataList.push_back(dataPack);
            }
        }
        else
        {
            printf((std::string(RED) +
                "[error] control_commands_map_ is empty, check if the protocol is load" + CLEANUP + "\n").c_str());
        }

        return DataList.size();
    }

    std::vector<std::pair<std::string, std::any>> retrieveFeedback(uint32_t ID, uint32_t DeviceId, const std::array<uint8_t, 8>& Raw,
        int ExtendedId = -1)
    {
        std::vector<std::pair<std::string, std::any>> feedbacks;
        if (feedbacks_map_)
        {
            for (const auto& param : *feedbacks_map_)
            {
                uint32_t addr = param.second.base_addr_;
                if (!param.second.is_absolute_base_)
                {
                    addr += param.second.device_id_shift_.dir_ == Shift::SHIFT_LEFT ?
                        (DeviceId << param.second.device_id_shift_.bits_) :
                        (DeviceId >> param.second.device_id_shift_.bits_);
                }
                if (addr == ID)
                {
                    if (param.second.extended_id_ && *param.second.extended_id_ != ExtendedId)
                    {
                        continue;
                    }

                    // NEW: bitpack decoding branch
                    if (param.second.is_bitpack_)
                    {
                        uint32_t totalBits = 0;
                        for (const auto& f : param.second.bit_fields_out_)
                        {
                            totalBits += f.bit_width_;
                        }
                        uint32_t numBytes = (totalBits + 7) / 8;

                        uint64_t acc = 0;
                        for (uint32_t i = 0; i < numBytes; ++i)
                        {
                            acc = (acc << 8) | uint64_t(Raw[param.second.start_byte_ + i]);
                        }

                        uint32_t pos = numBytes * 8;
                        for (const auto& f : param.second.bit_fields_out_)
                        {
                            pos -= f.bit_width_;
                            uint32_t raw = uint32_t((acc >> pos) & ((1u << f.bit_width_) - 1));
                            double norm = double(raw) / double((1u << f.bit_width_) - 1);
                            double val = norm * (f.max_ - f.min_) + f.min_;
                            feedbacks.push_back(std::pair<std::string, std::any>(f.name_, val));
                        }
                        continue;
                    }

                    if (param.second.data_type_ == FeedbackParam::int_data)
                    {
                        long value = 0;
                        size_t count = param.second.bytes_.size();
                        for (size_t i = 0; i < count - 1; ++i)
                        {
                            value |= uint8_t(Raw[param.second.bytes_[i].index_]) <<
                                param.second.bytes_[i].shift_.bits_;
                        }
                        value |= int8_t(Raw[param.second.bytes_[count - 1].index_]) <<
                            param.second.bytes_[count - 1].shift_.bits_;

                        value = value * param.second.multiplier_ / param.second.divider_ + param.second.additioner_;

                        feedbacks.push_back(std::pair<std::string, std::any>(param.first, value));
                    }
                    else if (param.second.data_type_ == FeedbackParam::float_data)
                    {
                        unsigned long value = 0;
                        size_t count = param.second.bytes_.size();
                        for (size_t i = 0; i < count; ++i)
                        {
                            value |= uint8_t(Raw[param.second.bytes_[i].index_]) <<
                                param.second.bytes_[i].shift_.bits_;
                        }

                        value = value * param.second.multiplier_ / param.second.divider_ + param.second.additioner_;

                        feedbacks.push_back(std::pair<std::string, std::any>(param.first, *(float*)&value));
                    }
                }
            }
        }
        else
        {
            printf((std::string(RED) +
                "[error] feedbacks_map_ is empty, check if the protocol is load" + CLEANUP + "\n").c_str());
        }

        return feedbacks;
    }

protected:
    static void loadStaticCommandsList(const YAML::Node& Node, const std::string& Key,
        std::unique_ptr<StaticCommandsList>& List)
    {
        const auto& commands = Node[Key];
        if (commands)
        {
            List = std::make_unique<StaticCommandsList>();

            for (const auto& command : commands)
            {
                List->push_back(StaticCommand());
                List->back().name_ = command["param"].as<std::string>();
                for (const auto& it : command["data"])
                {
                    List->back().data_.push_back(uint8_t(it.as<int>()));
                }
                List->back().base_addr_ = command["base_addr"].as<uint32_t>();
                const auto& absolute = command["absolute_base"];
                if (absolute)
                {
                    List->back().is_absolute_base_ = absolute.as<bool>();
                }
                const auto& variables = command["variables"];
                if (variables)
                {
                    for (const auto& it : variables)
                    {
                        List->back().variables_map_.emplace(
                            std::make_pair(it.second.as<std::string>(), uint8_t(it.first.as<int>())));
                    }
                }
                const auto& delay = command["delay"];
                if (delay)
                {
                    List->back().delay_ = 1000 * delay.as<int>();
                }
            }
        }
    }

    static void loadStaticCommands(const YAML::Node& Node, const std::string& Key,
        std::unique_ptr<StaticCommandsGroupsMap>& GroupMap, std::unique_ptr<StaticCommandsMap>& ItemMap)
    {
        const auto& items = Node[Key];
        if (items)
        {
            for (const auto& item : items)
            {
                const auto& group = item["group"];
                if (group)
                {
                    // commands list with group
                    if (!GroupMap)
                    {
                        GroupMap = std::make_unique<StaticCommandsGroupsMap>();
                    }

                    std::string groupName = group.as<std::string>();
                    StaticCommandsList list;

                    const auto& params = item["params"];
                    for (const auto& param : params)
                    {
                        list.push_back(StaticCommand());
                        list.back().name_ = param["param"].as<std::string>();
                        for (const auto& it : param["data"])
                        {
                            list.back().data_.push_back(uint8_t(it.as<int>()));
                        }
                        list.back().base_addr_ = param["base_addr"].as<uint32_t>();
                        const auto& absolute = param["absolute_base"];
                        if (absolute)
                        {
                            list.back().is_absolute_base_ = absolute.as<bool>();
                        }
                        const auto& variables = param["variables"];
                        if (variables)
                        {
                            for (const auto& it : variables)
                            {
                                list.back().variables_map_.emplace(
                                    std::make_pair(it.second.as<std::string>(), uint8_t(it.first.as<int>())));
                            }
                        }
                        const auto& delay = param["delay"];
                        if (delay)
                        {
                            list.back().delay_ = 1000 * delay.as<int>();
                        }
                    }

                    GroupMap->emplace(groupName, list);
                }
                else
                {
                    // commands list
                    if (!ItemMap)
                    {
                        ItemMap = std::make_unique<StaticCommandsMap>();
                    }
                
                    std::string name = item["param"].as<std::string>();
                    ItemMap->emplace(name, StaticCommand());
                    ItemMap->at(name).name_= name;
                    for (const auto& it : item["data"])
                    {
                        ItemMap->at(name).data_.push_back(uint8_t(it.as<int>()));
                    }
                    ItemMap->at(name).base_addr_ = item["base_addr"].as<uint32_t>();
                    const auto& absolute = item["absolute_base"];
                    if (absolute)
                    {
                        ItemMap->at(name).is_absolute_base_ = absolute.as<bool>();
                    }
                    const auto& variables = item["variables"];
                    if (variables)
                    {
                        for (const auto& it : variables)
                        {
                            ItemMap->at(name).variables_map_.emplace(
                                std::make_pair(it.second.as<std::string>(), uint8_t(it.first.as<int>())));
                        }
                    }
                    const auto& delay = item["delay"];
                    if (delay)
                    {
                        ItemMap->at(name).delay_ = 1000 * delay.as<int>();
                    }
                }
            }
#ifdef DEBUG
            for (const auto& grp : *GroupMap)
            {
                std::cout << "group " << grp.first << " with size " << grp.second.size() << std::endl;
                for (const auto& par : grp.second)
                {
                    std::cout << "param " << par.name_ << ", ";
                }
                std::cout << std::endl;
            }
#endif
        }
    }

    static void loadControlCommandsMap(const YAML::Node& Node, const std::string& Key,
        std::unique_ptr<ControlCommandsMap>& Map)
    {
        const auto& commands = Node[Key];
        if (commands)
        {
            Map = std::make_unique<ControlCommandsMap>();

            for (const auto& command : commands)
            {
                std::string name = command["command"].as<std::string>();
                Map->emplace(std::make_pair(name, ControlCommand()));

                Map->at(name).name_ = name;

                const auto& family = command["family"];
                if (family)
                {
                    Map->at(name).family_ = family.as<std::string>();
                }

                Map->at(name).base_addr_ = command["base_addr"].as<uint32_t>();

                const auto& absolute = command["absolute_base"];
                if (absolute)
                {
                    Map->at(name).is_absolute_base_ = absolute.as<bool>();
                }

                const auto& multiplier = command["multiplier"];
                if (multiplier)
                {
                    Map->at(name).multiplier_ = multiplier.as<double>();
                }

                const auto& divider = command["divider"];
                if (divider)
                {
                    Map->at(name).divider_ = divider.as<double>();
                }

                const auto& additioner = command["additioner"];
                if (additioner)
                {
                    Map->at(name).additioner_ = additioner.as<double>();
                }

                // NEW: bitpack encoding path — checked before the legacy per-byte "bytes" parsing
                const auto& encoding = command["encoding"];
                if (encoding && encoding.as<std::string>() == "bitpack")
                {
                    Map->at(name).is_bitpack_ = true;

                    const auto& startByte = command["start_byte"];
                    if (startByte)
                    {
                        Map->at(name).start_byte_ = uint8_t(startByte.as<int>());
                    }

                    const auto& fields = command["bit_fields"];
                    for (const auto& f : fields)
                    {
                        Protocol::BitFieldIn bf;
                        bf.source_ = f["source"].as<std::string>();
                        bf.bit_width_ = uint8_t(f["bit_width"].as<int>());
                        bf.min_ = f["min"].as<double>();
                        bf.max_ = f["max"].as<double>();
                        Map->at(name).bit_fields_.push_back(bf);
                    }

                    continue; // skip legacy per-byte parsing for this command
                }

                const auto& bytes = command["bytes"];
                // find the bytes length
                int bytesLen = 0;
                for (const auto& byte : bytes)
                {
                    int index = byte["byte"].as<int>();
                    if (index > bytesLen && index < 8)
                    {
                        bytesLen = index;
                    }
                }
                // construct control command map
                Map->at(name).data_.resize(++bytesLen);
                for (const auto& byte : bytes)
                {
                    int index = byte["byte"].as<int>();
                    try
                    {
                        Map->at(name).data_[index].source_ = uint8_t(byte["source"].as<int>());
#ifdef DEBUG
                        std::cout << "index " << index << " source is " <<
                            std::to_string(std::any_cast<uint8_t>(Map->at(name).data_[index].source_)) <<
                            " with type " << Map->at(name).data_[index].source_.type().name() << std::endl;
#endif
                    }
                    catch (const std::exception& e)
                    {
                        Map->at(name).data_[index].source_ = byte["source"].as<std::string>();
#ifdef DEBUG
                        std::cout << "index " << index << " source is " <<
                            std::any_cast<std::string>(Map->at(name).data_[index].source_) << " with type" <<
                                Map->at(name).data_[index].source_.type().name() << std::endl;
#endif
                    }

                    const auto& type = byte["data_type"];
                    if (type)
                    {
                        std::string typeStr = type.as<std::string>();
                        if (typeStr.find("int") != std::string::npos)
                        {
                            Map->at(name).data_[index].data_type_ = CommandByte::int_data;
                        }
                        else if (typeStr.find("float") != std::string::npos)
                        {
                            Map->at(name).data_[index].data_type_ = CommandByte::float_data;
                        }
                    }

                    const auto& cond = byte["condition"];
                    if (cond)
                    {
                        Map->at(name).data_[index].cond_ = std::make_shared<Condition>();
                        Map->at(name).data_[index].cond_->target_ = int8_t(cond["target"].as<int>());
#ifdef DEBUG
                        std::cout << "index " << index << " with cond target " <<
                            std::to_string(Map->at(name).data_[index].cond_->target_) << std::endl;
#endif
                        const auto& less = cond["less"];
                        if (less)
                        {
                            Map->at(name).data_[index].cond_->less_ = std::make_shared<uint8_t>(uint8_t(less.as<int>()));
#ifdef DEBUG
                            std::cout << "index " << index << " with cond less " <<
                                std::to_string(*Map->at(name).data_[index].cond_->less_) << std::endl;
#endif
                        }

                        const auto& equal = cond["equal"];
                        if (equal)
                        {
                            Map->at(name).data_[index].cond_->equal_ = std::make_shared<uint8_t>(uint8_t(equal.as<int>()));
#ifdef DEBUG
                            std::cout << "index " << index << " with cond equal " <<
                                std::to_string(*Map->at(name).data_[index].cond_->equal_) << std::endl;
#endif
                        }

                        const auto& great = cond["great"];
                        if (great)
                        {
                            Map->at(name).data_[index].cond_->great_ = std::make_shared<uint8_t>(uint8_t(great.as<int>()));
#ifdef DEBUG
                            std::cout << "index " << index << " with cond great " <<
                                std::to_string(*Map->at(name).data_[index].cond_->great_) << std::endl;
#endif
                        }
                    }

                    const auto& shift = byte["shift"];
                    if (shift)
                    {
                        Map->at(name).data_[index].shift_ = std::make_shared<Shift>();
                        Map->at(name).data_[index].shift_->bits_ = uint8_t(shift["bits"].as<int>());
                        Map->at(name).data_[index].shift_->dir_ = shift["dir"].as<std::string>() == "left" ?
                            Shift::SHIFT_LEFT : Shift::SHIFT_RIGHT;
#ifdef DEBUG
                        std::cout << "index " << index << " with " <<
                            (Map->at(name).data_[index].shift_->dir_ == Shift::SHIFT_LEFT ? "left" : "right") <<
                            " shift bits " << std::to_string(Map->at(name).data_[index].shift_->bits_) << std::endl;
#endif
                    }

                    const auto& absolute = byte["absolute_value"];
                    if (absolute)
                    {
                        Map->at(name).data_[index].is_absolute_ = absolute.as<bool>();
                    }

                    const auto& multiplier = byte["multiplier"];
                    if (multiplier)
                    {
                        Map->at(name).data_[index].multiplier_ = multiplier.as<double>();
                    }

                    const auto& divider = byte["divider"];
                    if (divider)
                    {
                        Map->at(name).data_[index].divider_ = divider.as<double>();
                    }

                    const auto& additioner = byte["additioner"];
                    if (additioner)
                    {
                        Map->at(name).data_[index].additioner_ = additioner.as<double>();
                    }
                }
            }
        }
    }

    static void loadFeedbacksMap(const YAML::Node& Node, const std::string& Key,
        std::unique_ptr<FeedbacksMap>& Map)
    {
        const auto& feedbacks = Node["feedbacks"];
        if (feedbacks)
        {
            Map = std::make_unique<FeedbacksMap>();

            for (const auto& param : feedbacks)
            {
                auto name = param["param"].as<std::string>();
                Map->emplace(std::make_pair(name, FeedbackParam()));

                Map->at(name).base_addr_ = param["base_addr"].as<uint32_t>();
                const auto& absolute =  param["absolute_base"];
                if (absolute)
                {
                    Map->at(name).is_absolute_base_ = absolute.as<bool>();
                }
                const auto& multiplier = param["multiplier"];
                if (multiplier)
                {
                    Map->at(name).multiplier_ = multiplier.as<double>();
                }
                const auto& divider = param["divider"];
                if (divider)
                {
                    Map->at(name).divider_ = divider.as<double>();
                }
                const auto& additioner = param["additioner"];
                if (additioner)
                {
                    Map->at(name).additioner_ = additioner.as<double>();
                }
                const auto& type = param["data_type"];
                if (type)
                {
                    std::string typeStr = type.as<std::string>();
                    if (typeStr.find("int") != std::string::npos)
                    {
                        Map->at(name).data_type_ = FeedbackParam::int_data;
                    }
                    else if (typeStr.find("float") != std::string::npos)
                    {
                        Map->at(name).data_type_ = FeedbackParam::float_data;
                    }
                }
                const auto& deviceIdShift = param["device_id_shift"];
                if (deviceIdShift)
                {
                    Map->at(name).device_id_shift_.bits_ = uint8_t(deviceIdShift["bits"].as<int>());
                    Map->at(name).device_id_shift_.dir_ = deviceIdShift["dir"].as<std::string>() == "left" ?
                        Shift::SHIFT_LEFT : Shift::SHIFT_RIGHT;
                }
                const auto& extendedId = param["extended_id"];
                if (extendedId)
                {
                    Map->at(name).extended_id_ = std::make_shared<int>(extendedId.as<int>());
                }

                // NEW: bitpack decoding path — checked before the legacy per-byte "bytes" parsing
                const auto& encoding = param["encoding"];
                if (encoding && encoding.as<std::string>() == "bitpack")
                {
                    Map->at(name).is_bitpack_ = true;

                    const auto& startByte = param["start_byte"];
                    if (startByte)
                    {
                        Map->at(name).start_byte_ = uint8_t(startByte.as<int>());
                    }

                    const auto& fields = param["bit_fields"];
                    for (const auto& f : fields)
                    {
                        Protocol::BitFieldOut bf;
                        bf.name_ = f["name"].as<std::string>();
                        bf.bit_width_ = uint8_t(f["bit_width"].as<int>());
                        bf.min_ = f["min"].as<double>();
                        bf.max_ = f["max"].as<double>();
                        Map->at(name).bit_fields_out_.push_back(bf);
                    }

                    continue; // skip legacy per-byte parsing for this feedback
                }

                auto bytes = param["bytes"];
                for (const auto& it : bytes)
                {
                    FeebackByte feed;
                    feed.index_ = uint8_t(it.first.as<int>());
                    feed.shift_.bits_ = uint8_t(it.second.as<int>());
                    Map->at(name).bytes_.push_back(feed);
                }
                // sort bytes with ascending order for determining the sign of bit shift
                std::sort(Map->at(name).bytes_.begin(), Map->at(name).bytes_.end(),
                    [](const FeebackByte& A, const FeebackByte& B)
                    {
                        return A.shift_.bits_ < B.shift_.bits_;
                    });
            }
#ifdef DEBUG
            for (const auto& param : *Map)
            {
                std::cout << "feedback param " << param.first << " with bit index ";
                for (const auto& bit : param.second.bytes_)
                {
                    std::cout << std::to_string(bit.index_) << " shift " <<
                        (bit.shift_.dir_ == Shift::SHIFT_LEFT ? "left " : "right ") <<
                        std::to_string(bit.shift_.bits_) << " bits" << std::endl;
                }
            }
#endif
        }
    }
};