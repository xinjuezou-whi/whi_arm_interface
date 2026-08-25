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
#include <errno.h>
#include <iostream>

// FIX: shared init used by both constructors. Zero every member struct so
// nothing read before open() (ioctl/bind fields, iov_/msg_ layout, etc.) is
// left as indeterminate stack/heap garbage, then copy Name into ifr_name
// bounded to IFNAMSIZ with a guaranteed null terminator -- replaces the
// previous unbounded strcpy(), which would silently overrun ifr_ (and
// whatever CanBus member follows it in memory) if Name.size() >= IFNAMSIZ.
void CanBus::initMembers(const char* Name)
{
    memset(&addr_, 0, sizeof(addr_));
    memset(&ifr_, 0, sizeof(ifr_));
    memset(&frame_, 0, sizeof(frame_));
    memset(&iov_, 0, sizeof(iov_));
    memset(&msg_, 0, sizeof(msg_));
    memset(ctrlmsg, 0, sizeof(ctrlmsg));
    memset(&event_pending_, 0, sizeof(event_pending_));

    if (Name != nullptr)
    {
        strncpy(ifr_.ifr_name, Name, IFNAMSIZ - 1);
        ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
    }
}

CanBus::CanBus(const char* Name)
{
    initMembers(Name);
}

CanBus::CanBus(const std::string& Name)
{
    initMembers(Name.c_str());
}

bool CanBus::open()
{
    if ((fd_epoll_ = epoll_create(1)) < 0)
    {
        printf((std::string(RED) + "[fatal] failed to create epoll" + CLEANUP + "\n").c_str());
        is_open_ = false;  // 修复：epoll 失败也要标记
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

    ioctl(if_obj_.socket_, SIOCGIFINDEX, &ifr_);

    memset(&addr_, 0, sizeof(addr_));
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    iov_.iov_base = &frame_;
    msg_.msg_name = &addr_;
    msg_.msg_iov = &iov_;
    msg_.msg_iovlen = 1;
    msg_.msg_control = &ctrlmsg;

    // NEW: enlarge the socket's receive buffer before bind(), giving the
    // kernel more headroom to queue incoming frames if this thread's read()
    // loop momentarily falls behind the bus rate (multiple joints sharing
    // one physical bus each open their own socket, so under load the kernel
    // buffer is the only thing standing between "briefly slow" and "frame
    // dropped").
    {
        int rcvbuf = 1 << 20; // 1MB
        setsockopt(if_obj_.socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    }

    // NEW (2026-08-21): apply a receive timeout so blocking read() calls on
    // this socket never wait forever. Without this, a reader thread that's
    // been told to stop (e.g. DriverDamiao::threadReadCan() observing
    // terminated_) has no way to wake up and notice that if no frame
    // happens to be arriving on the bus at that moment -- it can only find
    // out the next time a frame lands. On a quiet/idle bus that can be a
    // very long wait, which in turn stalls close()'s th_read_.join() and,
    // transitively, shutdown of the whole node.
    {
        struct timeval tv;
        tv.tv_sec = recv_timeout_ms_ / 1000;
        tv.tv_usec = (recv_timeout_ms_ % 1000) * 1000;
        setsockopt(if_obj_.socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    if (bind(if_obj_.socket_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0)
    {
        printf((std::string(RED) + "[fatal] cannot bind socket %d to %s" + CLEANUP + "\n").c_str(),
            if_obj_.socket_, ifr_.ifr_name);
        is_open_ = false;
        return false;
    }

    // NEW: this bus is confirmed running in CAN-FD mode (bitrate 1Mbps /
    // dbitrate 5Mbps, `ip -details link show` reports `can <FD>`). A raw CAN
    // socket defaults to classic-frame-only; without opting in via
    // CAN_RAW_FD_FRAMES, every FD frame arriving on the wire is silently
    // dropped at the socket layer -- no error, no warn, nothing queued for
    // recv. The read() call below then blocks forever waiting for a frame
    // that will never arrive (visible as every reader thread parked in
    // skb_wait_for_more_packets with zero throughput and zero errors).
    // This MUST be set before any read()/write() call; setsockopt() order
    // relative to CAN_RAW_FILTER below does not matter.
    {
        int enableFd = 1;
        if (setsockopt(if_obj_.socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
            &enableFd, sizeof(enableFd)) < 0)
        {
            printf((std::string(YELLOW) + "[warn] failed to enable CAN_RAW_FD_FRAMES on %s" + CLEANUP + "\n").c_str(),
                ifr_.ifr_name);
        }
    }

    // NEW: apply a kernel-side receive filter AFTER bind() (standard SocketCAN
    // ordering), so that from this point on the socket only sees frames
    // addressed to recv_filter_id_. Without this, every CanBus instance on a
    // shared bus (e.g. one per joint, all bound to the same physical
    // interface) sees ALL frames on that bus -- including its own command
    // loopback and every other joint's traffic -- which under load can
    // starve/drop the specific response frame this instance actually cares
    // about.
    if (recv_filter_id_.has_value())
    {
        struct can_filter filter;
        filter.can_id = recv_filter_id_.value();
        filter.can_mask = CAN_SFF_MASK; // match full 11-bit standard frame ID
        if (setsockopt(if_obj_.socket_, SOL_CAN_RAW, CAN_RAW_FILTER,
            &filter, sizeof(filter)) < 0)
        {
            printf((std::string(YELLOW) + "[warn] failed to set can filter (id 0x%03X) on %s" + CLEANUP + "\n").c_str(),
                recv_filter_id_.value(), ifr_.ifr_name);
        }
    }

    is_open_ = true;  // 修复：成功时设置 is_open_
    return true;
}

bool CanBus::isOpen()
{
    return is_open_;
}

bool CanBus::open(const std::string& Name)
{
    // FIX: bounded copy, same reasoning as initMembers()/constructors --
    // the previous strcpy() here had no length check either.
    strncpy(ifr_.ifr_name, Name.c_str(), IFNAMSIZ - 1);
    ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
    return open();  // 修复：直接复用 open()，is_open_ 已在其中设置，无需再赋值
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
    }

    if (fd_epoll_ >= 0)
    {
        ::close(fd_epoll_);
    }

    is_open_ = false;
}

ssize_t CanBus::read(unsigned int& ID, unsigned char* Data)
{
    ssize_t len = -1;

    if (if_obj_.socket_ >= 0)
    {
        // NEW: read into a canfd_frame-sized buffer so this single call
        // transparently accepts whatever lands on the CAN_RAW_FD_FRAMES-
        // enabled socket: a classic CAN frame (kernel returns exactly
        // CAN_MTU bytes) or a CAN-FD frame (kernel returns exactly
        // CANFD_MTU bytes). CAN_MTU/CANFD_MTU are the two values the
        // kernel is guaranteed to hand back on a well-formed read, so
        // branching on the returned byte count is the reliable way to
        // tell which layout we got -- no guessing based on can_id or flags.
        struct canfd_frame frame;
        ssize_t nbytes = ::read(if_obj_.socket_, &frame, sizeof(struct canfd_frame));

        if (nbytes == CAN_MTU)
        {
            // classic frame landed; canfd_frame's leading fields
            // (can_id/len/flags/data...) are layout-compatible with the
            // leading fields of can_frame for this purpose, but re-read
            // it explicitly as can_frame to keep can_dlc semantics correct.
            struct can_frame* classic = reinterpret_cast<struct can_frame*>(&frame);
            ID = classic->can_id;
            len = classic->can_dlc;
            // FIX: can_dlc is a single byte straight off the wire. The
            // kernel is expected to hand back a value in [0, 8] for a
            // classic frame, but this was previously trusted unconditionally
            // -- a malformed/adversarial frame (or a kernel/driver bug) with
            // can_dlc > 8 would memcpy past the caller's fixed 8-byte Data
            // buffer (e.g. DriverDamiao::threadReadCan()'s stack-local
            // `unsigned char raw[8]`), corrupting whatever follows it on the
            // stack without failing immediately. Clamp defensively, same as
            // the FD branch below already does.
            if (len > 8)
            {
                len = 8;
            }
            memcpy(Data, classic->data, len);
        }
        else if (nbytes == CANFD_MTU)
        {
            ID = frame.can_id;
            len = frame.len;
            // Damiao MIT-mode payload is always <=8 bytes; callers (e.g.
            // DriverDamiao::threadReadCan) pass an 8-byte Data buffer, so
            // clamp defensively even though we don't expect FD's up-to-64
            // byte payload in this protocol.
            if (len > 8)
            {
                len = 8;
            }
            memcpy(Data, frame.data, len);
        }
        else
        {
            // NEW (2026-08-21): with SO_RCVTIMEO now set in open(), a timed
            // out read is an expected, routine outcome (it happens every
            // recv_timeout_ms_ whenever the bus is momentarily idle), not
            // an error -- so it must NOT print the "[warn] failed to read"
            // below, or a reader thread on a quiet bus would spam that
            // warning roughly 10x/second forever. Distinguish "genuinely
            // timed out" (errno EAGAIN/EWOULDBLOCK) from any other real
            // read failure, which still deserves the warning.
            if (nbytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                return -1;
            }

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
        // FIX: can_frame.data is a fixed 8-byte array (CAN_MAX_DLEN). Len
        // previously came straight from the caller (e.g.
        // DriverDamiao::actuate() -> Protocol::composeCommand()'s bitpack
        // path, whose output size is derived from protocol yaml config) with
        // no bound check before the memcpy below. A misconfigured yaml
        // (bit_fields totalling more than (8 - start_byte) bytes) would
        // silently produce a >8-byte payload and overflow frame.data on the
        // stack. Reject rather than truncate -- truncating would silently
        // send a corrupt/short command to the joint, which is worse than
        // failing loudly here.
        if (Data == nullptr || Len > CAN_MAX_DLEN)
        {
            printf((std::string(RED) + "[error] write len %zu exceeds CAN_MAX_DLEN (%d) on %s, refusing to send" +
                CLEANUP + "\n").c_str(), Len, CAN_MAX_DLEN, ifr_.ifr_name);
            return false;
        }

        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = ID;
        if (isExtended(frame.can_id))
        {
            frame.can_id |= CAN_EFF_FLAG;
        }
        frame.can_dlc = static_cast<unsigned char>(Len);
        memcpy(frame.data, Data, frame.can_dlc);

        // NOTE: writing a classic can_frame (CAN_MTU bytes) on a
        // CAN_RAW_FD_FRAMES-enabled socket is valid and sent as a classic
        // frame on the wire -- no change needed here for the FD fix.
        // Damiao MIT control frames are 8-byte classic frames by protocol,
        // so this stays as-is.
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
            // FIX: frame_.len for a canfd_frame can be up to 64 (CANFD_MAX_DLEN);
            // this function's contract (mirroring read()) assumes an 8-byte
            // Data buffer from the caller. Clamp defensively -- same reasoning
            // as read()'s FD branch.
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