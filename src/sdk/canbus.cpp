/******************************************************************
CanBus class for can-bus communication

Features:
- SocketCAN
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-08-21: FIX: see canbus.h changelog for the same date -- bounded
            copies in write()/read(), bounded+zero-inited constructors.
******************************************************************/
#include "whi_arm_interface/canbus.h"
#include "whi_arm_interface/printf_color.h"

#include <string.h>
#include <unistd.h>
#include <iostream>

CanBus::CanBus(const char* Name)
{
    strncpy(ifr_.ifr_name, Name, IFNAMSIZ - 1);
    ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
}

CanBus::CanBus(const std::string& Name)
{
    strncpy(ifr_.ifr_name, Name.c_str(), IFNAMSIZ - 1);
    ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
}

bool CanBus::open()
{
    if ((fd_epoll_ = epoll_create(1)) < 0)
    {
        printf((std::string(RED) + "[fatal] failed to create epoll" + CLEANUP + "\n").c_str());
        is_open_ = false;
        return false;
    }

    if ((if_obj_.socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        printf((std::string(RED) + "[fatal] cannot open socket for PF_CAN" + CLEANUP + "\n").c_str());
        is_open_ = false;
        return false;
    }

    event_setup_.data.ptr = &if_obj_;
    if (epoll_ctl(fd_epoll_, EPOLL_CTL_ADD, if_obj_.socket_, &event_setup_))
    {
        printf((std::string(RED) + "[fatal] failed to add socket to epoll" + CLEANUP + "\n").c_str());
        is_open_ = false;
        return false;
    }

    if ((fd_shutdown_ = eventfd(0, EFD_NONBLOCK)) >= 0)
    {
        struct epoll_event evShutdown{};
        evShutdown.events = EPOLLIN;
        evShutdown.data.ptr = nullptr;
        if (epoll_ctl(fd_epoll_, EPOLL_CTL_ADD, fd_shutdown_, &evShutdown))
        {
            printf((std::string(YELLOW) + "[warn] failed to add shutdown eventfd to epoll" + CLEANUP + "\n").c_str());
            ::close(fd_shutdown_);
            fd_shutdown_ = -1;
        }
    }
    else
    {
        printf((std::string(YELLOW) + "[warn] failed to create shutdown eventfd" + CLEANUP + "\n").c_str());
    }

    ioctl(if_obj_.socket_, SIOCGIFINDEX, &ifr_);

    memset(&addr_, 0, sizeof(addr_));
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    iov_.iov_base = &frame_;
    msg_.msg_name = &addr_;
    msg_.msg_iov = &iov_;
    msg_.msg_iovlen = 1;
    msg_.msg_control = &ctrlmsg;

    {
        int rcvbuf = 1 << 20; // 1MB
        setsockopt(if_obj_.socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    }

    if (bind(if_obj_.socket_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0)
    {
        printf((std::string(RED) + "[fatal] cannot bind socket %d to %s" + CLEANUP + "\n").c_str(),
            if_obj_.socket_, ifr_.ifr_name);
        is_open_ = false;
        return false;
    }

    {
        int enableFd = 1;
        if (setsockopt(if_obj_.socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
            &enableFd, sizeof(enableFd)) < 0)
        {
            printf((std::string(YELLOW) + "[warn] failed to enable CAN_RAW_FD_FRAMES on %s" + CLEANUP + "\n").c_str(),
                ifr_.ifr_name);
        }
    }

    if (recv_filter_id_.has_value())
    {
        struct can_filter filter;
        filter.can_id = recv_filter_id_.value();
        filter.can_mask = CAN_SFF_MASK;
        if (setsockopt(if_obj_.socket_, SOL_CAN_RAW, CAN_RAW_FILTER,
            &filter, sizeof(filter)) < 0)
        {
            printf((std::string(YELLOW) + "[warn] failed to set can filter (id 0x%03X) on %s" + CLEANUP + "\n").c_str(),
                recv_filter_id_.value(), ifr_.ifr_name);
        }
    }

    is_open_ = true;
    return true;
}

bool CanBus::isOpen()
{
    return is_open_;
}

bool CanBus::open(const std::string& Name)
{
    strncpy(ifr_.ifr_name, Name.c_str(), IFNAMSIZ - 1);
    ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
    return open();
}

void CanBus::close()
{
    if (if_obj_.socket_ >= 0)
    {
        if (::close(if_obj_.socket_) < 0)
        {
            printf((std::string(YELLOW) + "[warn] failed to close socket %d" + CLEANUP + "\n").c_str(),
                if_obj_.socket_);
        }
        if_obj_.socket_ = -1;
    }

    if (fd_shutdown_ >= 0)
    {
        ::close(fd_shutdown_);
        fd_shutdown_ = -1;
    }

    if (fd_epoll_ >= 0)
    {
        ::close(fd_epoll_);
        fd_epoll_ = -1;
    }

    is_open_ = false;
}

void CanBus::requestShutdown()
{
    if (fd_shutdown_ >= 0)
    {
        uint64_t one = 1;
        if (::write(fd_shutdown_, &one, sizeof(one)) < 0)
        {
            printf((std::string(YELLOW) + "[warn] failed to write shutdown eventfd" + CLEANUP + "\n").c_str());
        }
    }
}

ssize_t CanBus::read(unsigned int& ID, unsigned char* Data)
{
    ssize_t len = -1;

    if (if_obj_.socket_ >= 0)
    {
        struct canfd_frame frame;
        ssize_t nbytes = ::read(if_obj_.socket_, &frame, sizeof(struct canfd_frame));

        if (nbytes == CAN_MTU)
        {
            struct can_frame* classic = reinterpret_cast<struct can_frame*>(&frame);
            ID = classic->can_id;
            len = classic->can_dlc;
            memcpy(Data, classic->data, len);
        }
        else if (nbytes == CANFD_MTU)
        {
            ID = frame.can_id;
            len = frame.len;
            if (len > 8)
            {
                len = 8;
            }
            memcpy(Data, frame.data, len);
        }
        else
        {
            len = -1;
        }

        if (len > 0)
        {
#ifdef DEBUG
            printf("read ID: 0x%03X [%d] ", ID, (int)len);
            for (int i = 0; i < len; ++i)
            {
                printf("0x%02X ", Data[i]);
            }
            printf("\n");
#endif
        }
        else
        {
            printf((std::string(YELLOW) + "[warn] failed to read from %s" + CLEANUP + "\n").c_str(),
                ifr_.ifr_name);
        }
    }

    return len;
}

bool CanBus::write(unsigned int ID, size_t Len, const unsigned char* Data)
{
    if (if_obj_.socket_ >= 0)
    {
        // NEW: struct can_frame::data is a fixed 8-byte array (CAN_MAX_DLEN).
        // Len is ultimately protocol-yaml-controlled -- StaticCommand::data_
        // is built by push_back'ing every entry of a "data:" list with no
        // upper-bound check, and the bitpack encode path in
        // Protocol::composeCommand derives length from configured
        // start_byte_/bit_fields_ with no clamp either. A malformed/typo'd
        // protocol yaml (one extra "data:" entry, or a bitpack field
        // misconfigured to overflow 8 bytes) previously overflowed the
        // fixed-size `frame.data` (a stack-local struct in this function)
        // via memcpy. Clamp defensively and warn loudly instead of writing
        // out of bounds.
        if (Len > CAN_MAX_DLEN)
        {
            printf((std::string(RED) +
                "[error] CanBus::write len %zu exceeds classic CAN frame max %d on %s, clamped" +
                CLEANUP + "\n").c_str(), Len, CAN_MAX_DLEN, ifr_.ifr_name);
            Len = CAN_MAX_DLEN;
        }

        struct can_frame frame;
        frame.can_id = ID;
        if (isExtended(frame.can_id))
        {
            frame.can_id |= CAN_EFF_FLAG;
        }
        frame.can_dlc = static_cast<unsigned char>(Len);
        memcpy(frame.data, Data, frame.can_dlc);

        if (::write(if_obj_.socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            printf((std::string(YELLOW) + "[warn] failed to write %s" + CLEANUP + "\n").c_str(),
                ifr_.ifr_name);
            return false;
        }

#ifdef DEBUG
        printf("sent ID: 0x%03X [%d] ", frame.can_id, frame.can_dlc);
        for (int i = 0; i < frame.can_dlc; ++i)
        {
            printf("0x%02X ", Data[i]);
        }
        printf("\n");
#endif

        return true;
    }

    return false;
}

std::size_t CanBus::eventTriggered(unsigned int& ID, unsigned char* Data)
{
    std::size_t readCount = 0;

    if (epoll_wait(fd_epoll_, &event_pending_, 1, -1) > 0)
    {
        if (event_pending_.data.ptr == nullptr)
        {
            uint64_t discard;
            ::read(fd_shutdown_, &discard, sizeof(discard));
            return 0;
        }

        struct IfInfo* obj = (IfInfo*)event_pending_.data.ptr;
        char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3 * sizeof(struct timespec) + sizeof(__u32))];
        iov_.iov_len = sizeof(frame_);
        msg_.msg_namelen = sizeof(addr_);
        msg_.msg_controllen = sizeof(ctrlmsg);
        msg_.msg_flags = 0;

        ssize_t nbytes = recvmsg(obj->socket_, &msg_, 0);
        if (nbytes > 0)
        {
            readCount = nbytes;

            if (frame_.can_id & CAN_ERR_FLAG)
            {
                ID = frame_.can_id & (CAN_ERR_MASK | CAN_ERR_FLAG);
            }
            else if (frame_.can_id & CAN_EFF_FLAG)
            {
                ID = frame_.can_id & CAN_EFF_MASK;
            }
            else
            {
                ID = frame_.can_id & CAN_SFF_MASK;
            }

            std::size_t copyLen = frame_.len;
            if (copyLen > 8)
            {
                copyLen = 8;
            }
            memcpy(Data, frame_.data, copyLen);
        }
    }

    fflush(stdout);
    return readCount;
}

bool CanBus::isExtended(unsigned int ID) const
{
    return (ID & 0xffff0000) == 0 ? false : true;
}