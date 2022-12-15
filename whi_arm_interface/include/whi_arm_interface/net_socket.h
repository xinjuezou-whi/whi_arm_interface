/******************************************************************
socket facilities

Features:
- stream socket facilities
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2018-10-24: Initial version
2022-12-15: Refactored with RAII machanism
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <stdint.h>
#include <vector>
#include <memory>

#define RESULT_OK                    0
#define RESULT_FAIL_BIT              0x80000000
#define RESULT_ALREADY_DONE          0x20
#define RESULT_INVALID_DATA          (0x8000 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_FAIL        (0x8001 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_TIMEOUT     (0x8002 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_STOP        (0x8003 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_NOT_SUPPORT (0x8004 | RESULT_FAIL_BIT)
#define RESULT_FORMAT_NOT_SUPPORT    (0x8005 | RESULT_FAIL_BIT)
#define RESULT_INSUFFICIENT_MEMORY   (0x8006 | RESULT_FAIL_BIT)

struct sockaddr_storage;

class SocketAddress
{
public:
    enum AddressType
    {
        ADDRESS_TYPE_UNSPEC = 0,
        ADDRESS_TYPE_INET = 1,
        ADDRESS_TYPE_INET6 = 2,
    };

public:
    SocketAddress();
    SocketAddress(const std::string& Addr, int Port, AddressType Type = ADDRESS_TYPE_INET);
    SocketAddress(const SocketAddress& Src);
    SocketAddress& operator=(const SocketAddress&);
    virtual ~SocketAddress() = default;

protected:
    SocketAddress(std::unique_ptr<sockaddr_storage> PlatformData);

public:
    virtual int getPort() const;
    virtual uint32_t setPort(int Port);

    virtual uint32_t setAddressFromString(const std::string& Addr, AddressType Type = ADDRESS_TYPE_INET);
    virtual uint32_t getAddressAsString(std::string& Addr) const;
    virtual AddressType getAddressType() const;
    virtual uint32_t getRawAddress(std::string& Addr) const;

    virtual void setLoopbackAddress(AddressType Type = ADDRESS_TYPE_INET);
    virtual void setBroadcastAddressIPv4();
    virtual void setAnyAddress(AddressType Type = ADDRESS_TYPE_INET);

    const std::shared_ptr<sockaddr_storage> getPlatformData() const;

public:
    static size_t LoopUpHostName(const std::string& HostName, const std::string& SeviceName, std::vector<SocketAddress>& AddressPool,
        bool PerformDNS = true, AddressType Type = ADDRESS_TYPE_INET);

protected:
    std::shared_ptr<sockaddr_storage> platform_data_{ nullptr };
};



class SocketBase
{
public:
    enum SocketFamilyType
    {
        SOCKET_FAMILY_INET = 0,
        SOCKET_FAMILY_INET6 = 1,
        SOCKET_FAMILY_RAW = 2
    };

    enum SocketDirectionMask
    {
        SOCKET_DIR_RD = 0x01,
        SOCKET_DIR_WR = 0x02,
        SOCKET_DIR_BOTH = (SOCKET_DIR_RD | SOCKET_DIR_WR)
    };

    enum { DEFAULT_SOCKET_TIMEOUT = 10000 }; // 10sec

public:
    SocketBase() = default;
    virtual ~SocketBase() = default;

public:
    virtual uint32_t setTimeout(uint32_t Timeout, SocketDirectionMask Mask = SOCKET_DIR_BOTH) = 0;
    virtual uint32_t waitforSent(uint32_t Timeout = DEFAULT_SOCKET_TIMEOUT) = 0;
    virtual uint32_t waitforData(uint32_t Timeout = DEFAULT_SOCKET_TIMEOUT) = 0;

protected:
    virtual uint32_t bind(const SocketAddress& Addr) = 0;
    virtual uint32_t getLocalAddress(SocketAddress& Addr) = 0;
};



class StreamSocket : public SocketBase
{
public:
    enum { MAX_BACKLOG = 128 };

public:
    StreamSocket(SocketFamilyType Family = SOCKET_FAMILY_INET);
    virtual ~StreamSocket();

public:
    // overrides
    virtual uint32_t setTimeout(uint32_t Timeout, SocketDirectionMask Mask = SOCKET_DIR_BOTH);
    virtual uint32_t waitforSent(uint32_t Timeout = DEFAULT_SOCKET_TIMEOUT);
    virtual uint32_t waitforData(uint32_t Timeout = DEFAULT_SOCKET_TIMEOUT);
    // specific
    virtual uint32_t connect(const SocketAddress& AddrPack);
    virtual uint32_t listen(int Backlog = MAX_BACKLOG);
    virtual std::shared_ptr<StreamSocket> accept(std::shared_ptr<SocketAddress> AddrPack = nullptr);
    virtual uint32_t waitforIncomingConnection(uint32_t Timeout = DEFAULT_SOCKET_TIMEOUT);
    virtual uint32_t send(const void* Buffer, size_t Len);
    virtual uint32_t recv(void* Buffer, size_t Len, size_t& RecvLen);
    virtual uint32_t getPeerAddress(SocketAddress& AddrPack);
    virtual uint32_t shutdown(SocketDirectionMask Mask);
    virtual uint32_t enableKeepAlive(bool Enable = true);
    virtual uint32_t enableNoDelay(bool Enable = true);

protected:
    // overrides
    virtual uint32_t bind(const SocketAddress& Addr);
    virtual uint32_t getLocalAddress(SocketAddress& Addr);

private:
    int socket_fd_{ -1 };
};
