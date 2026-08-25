/******************************************************************
CanBus class for can-bus communication

Features:
- SocketCAN
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2021-07-03: Initial version
2021-08-29: Update declaration based on new coding formatting
2024-01-22: Add extended frame support
2026-08-13: Add optional recv-side CAN_RAW_FILTER support so multiple
            CanBus instances sharing the same physical bus each only
            receive frames addressed to them (kernel-side filtering,
            avoids each socket being flooded by every other joint's
            command loopback / other joints' responses on the bus)
2026-08-13: Add CAN_RAW_FD_FRAMES support -- bus is confirmed running in
            CAN-FD mode (bitrate 1Mbps / dbitrate 5Mbps). Without opting
            the socket into FD frames, the kernel silently drops every
            frame at the socket layer: no error, no warn, read() just
            blocks forever in skb_wait_for_more_packets. read() is
            updated accordingly to accept either classic or FD frame
            sizes off the wire.
2026-08-21: Add SO_RCVTIMEO recv timeout (configurable via
            setRecvTimeoutMs(), defaults to 100ms if never set). Without
            this, read() blocked indefinitely whenever no frame was
            arriving on the bus -- harmless during normal operation, but
            it meant a reader thread waiting to observe a "please stop"
            flag (e.g. DriverDamiao::threadReadCan() checking
            terminated_) could never wake up to notice it, which stalled
            shutdown (see DriverDamiao::close()'s th_read_.join()) long
            enough to blow past ros2 launch's 5s SIGINT grace period and
            get SIGTERM-killed before the arm was ever de-energized.
2026-08-21: FIX: bound-check all raw-buffer copies in/out of the fixed
            8-byte can_frame.data. Neither write() nor the classic-frame
            branch of read() previously clamped the copy length, so a
            misconfigured protocol yaml (composeCommand() producing a
            >8-byte payload) or a malformed/adversarial frame on the bus
            (an out-of-range can_dlc) could write past frame.data -- a
            stack buffer overflow that corrupts adjacent stack memory
            without failing immediately, surfacing later as an unrelated
            "free(): invalid size" abort. Also replaced the unbounded
            strcpy() in both constructors with a bounded, always-
            null-terminated copy into ifr_.ifr_name (IFNAMSIZ), and
            zero-initialize all members so nothing is read uninitialized
            before open() runs.
******************************************************************/
#pragma once
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <memory>
#include <optional>

class CanBus
{
public:
	using SharedPtr = std::shared_ptr<CanBus>;

public:
	CanBus() = delete;
	CanBus(const char* Name);
	CanBus(const std::string& Name);
	~CanBus() { close(); };

public:
	bool open();
	bool open(const std::string& Name);
	bool isOpen();
	void close();
	ssize_t read(unsigned int& ID, unsigned char* Data);
	bool write(unsigned int ID, size_t Len, const unsigned char* Data);
	std::size_t eventTriggered(unsigned int& ID, unsigned char* Data);
	bool isExtended(unsigned int ID) const;
	// NEW: set a kernel-side receive filter so this socket only gets frames
	// with can_id == Id (standard frame, full 11-bit mask). Must be called
	// BEFORE open()/open(Name) for it to take effect; calling it after the
	// socket is already open has no effect until the next open().
	void setRecvFilter(canid_t Id) { recv_filter_id_ = Id; }
	// NEW: remove any previously set filter (socket will receive all frames
	// on the bus again after the next open())
	void clearRecvFilter() { recv_filter_id_.reset(); }
	// NEW: configure the SO_RCVTIMEO applied to the socket in open(). Must
	// be called BEFORE open()/open(Name) for it to take effect. If never
	// called, open() falls back to a 100ms default -- read() is never
	// allowed to block forever.
	void setRecvTimeoutMs(int Ms) { recv_timeout_ms_ = Ms; }

protected:
	struct IfInfo // bundled information per open socket
	{
		int socket_{ -1 };
		__u32 dropcnt_{ 0 };
		__u32 last_dropcnt_{ 0 };
	};

protected:
	// FIX: shared helper for both constructors -- bounded name copy plus
	// zero-initializing every member that open()/read()/write() touch, so
	// nothing is left uninitialized (and thus undefined) before open() runs.
	void initMembers(const char* Name);

protected:
	struct sockaddr_can addr_;
	struct ifreq ifr_;
	struct IfInfo if_obj_;
	int fd_epoll_{ -1 };
	bool is_open_{ false };
	struct canfd_frame frame_;
	struct iovec iov_;
	struct msghdr msg_;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3 * sizeof(struct timespec) + sizeof(__u32))];
	struct epoll_event event_pending_;
	struct epoll_event event_setup_ = { .events = EPOLLIN }; // prepare the common part
	// NEW: optional recv-side filter id, applied in open() via CAN_RAW_FILTER
	std::optional<canid_t> recv_filter_id_{ std::nullopt };
	// NEW: recv timeout applied in open() via SO_RCVTIMEO, default 100ms
	int recv_timeout_ms_{ 100 };
};