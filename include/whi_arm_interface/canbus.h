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
2026-08-26: Add shutdown eventfd registered into the same epoll instance
            as the CAN socket. epoll_wait() in eventTriggered() was
            waiting with an infinite timeout (-1) and had no way to be
            woken up from another thread, so a caller's read thread
            parked in eventTriggered() could never observe a shutdown
            flag being set elsewhere -- join() on that thread would hang
            forever. requestShutdown() lets close()/the owning driver
            wake the blocked epoll_wait() on demand instead of guessing
            a timeout value.
******************************************************************/
#pragma once
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
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
	// returns 0 when woken by requestShutdown() (no CAN data available in
	// that case) or when nbytes read <= 0; otherwise returns nbytes read
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
	// NEW: wake up a thread currently blocked in eventTriggered()'s
	// epoll_wait(). Safe to call from any thread. No-op if the eventfd
	// was never created (open() not called, or eventfd() failed).
	void requestShutdown();

protected:
	struct IfInfo // bundled information per open socket
	{
		int socket_{ -1 };
		__u32 dropcnt_{ 0 };
		__u32 last_dropcnt_{ 0 };
	};

protected:
	struct sockaddr_can addr_;
	struct ifreq ifr_;
	struct IfInfo if_obj_;
	int fd_epoll_{ -1 };
	int fd_shutdown_{ -1 }; // NEW: eventfd used to interrupt epoll_wait()
	bool is_open_{ false };
	struct canfd_frame frame_;
	struct iovec iov_;
	struct msghdr msg_;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3 * sizeof(struct timespec) + sizeof(__u32))];
	struct epoll_event event_pending_;
	struct epoll_event event_setup_ = { .events = EPOLLIN }; // prepare the common part
	// NEW: optional recv-side filter id, applied in open() via CAN_RAW_FILTER
	std::optional<canid_t> recv_filter_id_{ std::nullopt };
};