/******************************************************************
socket facilities

Features:
- stream socket facilities
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_arm_interface/net_socket.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <cassert>

int halAddrTypeToOSType(SocketAddress::AddressType Type)
{
    switch (Type)
    {
    case SocketAddress::ADDRESS_TYPE_INET:
        return AF_INET;
    case SocketAddress::ADDRESS_TYPE_INET6:
        return AF_INET6;
    case SocketAddress::ADDRESS_TYPE_UNSPEC:
        return AF_UNSPEC;
    default:
        assert(!"should not reach here");
        return AF_UNSPEC;
    }
}

int socketHalFamilyToOSFamily(SocketBase::SocketFamilyType Family)
{
    switch (Family)
    {
    case SocketBase::SOCKET_FAMILY_INET:
        return AF_INET;
    case SocketBase::SOCKET_FAMILY_INET6:
        return AF_INET6;
    case SocketBase::SOCKET_FAMILY_RAW:
        return AF_PACKET;
    default:
        assert(!"should not reach here");
        return AF_INET; // force treating as IPv4 in release mode
    }
}

SocketAddress::SocketAddress()
{
    platform_data_ = std::make_shared<sockaddr_storage>();
    memset(platform_data_.get(), 0, sizeof(sockaddr_storage));

    platform_data_->ss_family = AF_INET;
}

SocketAddress::SocketAddress(const std::string& Addr, int Port, AddressType Type)
{
    platform_data_ = std::make_shared<sockaddr_storage>();
    memset(platform_data_.get(), 0, sizeof(sockaddr_storage));

    // default to ipv4 in case the following operation fails
    platform_data_->ss_family = AF_INET;

    setAddressFromString(Addr, Type);
    setPort(Port);
}

SocketAddress::SocketAddress(const SocketAddress& Src)
{
    *this = Src;
}

SocketAddress& SocketAddress::operator=(const SocketAddress& Src)
{
    if (this != &Src)
    {
        memcpy(platform_data_.get(), Src.platform_data_.get(), sizeof(sockaddr_storage));
    }
    
    return *this;
}

SocketAddress::SocketAddress(std::unique_ptr<sockaddr_storage> PlatformData)
    : platform_data_(std::move(PlatformData)) {}

SocketAddress::AddressType SocketAddress::getAddressType() const
{
    switch (platform_data_->ss_family)
    {
    case AF_INET:
        return ADDRESS_TYPE_INET;
    case AF_INET6:
        return ADDRESS_TYPE_INET6;
    default:
        assert(!"should not reach here");
        return ADDRESS_TYPE_INET;
    }
}

int SocketAddress::getPort() const
{
    switch (getAddressType())
    {
    case ADDRESS_TYPE_INET:
        return (int)ntohs(reinterpret_cast<const sockaddr_in*>(platform_data_.get())->sin_port);
    case ADDRESS_TYPE_INET6:
        return (int)ntohs(reinterpret_cast<const sockaddr_in6*>(platform_data_.get())->sin6_port);
    default:
        return 0;
    }
}

uint32_t SocketAddress::setPort(int Port)
{
    switch (getAddressType())
    {
    case ADDRESS_TYPE_INET:
        reinterpret_cast<sockaddr_in*>(platform_data_.get())->sin_port = htons((short)Port);
        break;
    case ADDRESS_TYPE_INET6:
        reinterpret_cast<sockaddr_in6*>(platform_data_.get())->sin6_port = htons((short)Port);
        break;
    default:
        return RESULT_OPERATION_FAIL;
    }
    return RESULT_OK;
}

uint32_t SocketAddress::setAddressFromString(const std::string& Addr, SocketAddress::AddressType Type)
{
    int ans = 0;
    int prevPort = getPort();
    switch (Type)
    {
    case ADDRESS_TYPE_INET:
        platform_data_->ss_family = AF_INET;
        ans = inet_pton(AF_INET, Addr.c_str(),
            &reinterpret_cast<sockaddr_in*>(platform_data_.get())->sin_addr);
        break;
    case ADDRESS_TYPE_INET6:
        platform_data_->ss_family = AF_INET6;
        ans = inet_pton(AF_INET6, Addr.c_str(),
            &reinterpret_cast<sockaddr_in6*>(platform_data_.get())->sin6_addr);
        break;
    default:
        return RESULT_INVALID_DATA;
    }
    setPort(prevPort);

    return ans <= 0 ? RESULT_INVALID_DATA : RESULT_OK;
}

uint32_t SocketAddress::getAddressAsString(std::string& Addr) const
{
    int net_family = platform_data_->ss_family;
    const char* ans = nullptr;
    Addr.resize(256);
    switch (net_family)
    {
    case AF_INET:
        ans = inet_ntop(net_family, &reinterpret_cast<const sockaddr_in*>(platform_data_.get())->sin_addr,
            Addr.data(), Addr.length());
        break;
    case AF_INET6:
        ans = inet_ntop(net_family, &reinterpret_cast<const sockaddr_in6*>(platform_data_.get())->sin6_addr,
            Addr.data(), Addr.length());
        break;
    }

    return ans ? RESULT_OK : RESULT_OPERATION_FAIL;
}

uint32_t SocketAddress::getRawAddress(std::string& Addr) const
{
    switch (getAddressType())
    {
    case ADDRESS_TYPE_INET:
        Addr.resize(sizeof(in_addr::s_addr) + 1);
        memcpy(Addr.data(), &reinterpret_cast<const sockaddr_in*>(platform_data_.get())->sin_addr.s_addr,
            sizeof(reinterpret_cast<const sockaddr_in*>(platform_data_.get())->sin_addr.s_addr));
        break;
    case ADDRESS_TYPE_INET6:
        Addr.resize(sizeof(in6_addr::s6_addr) + 1);
        memcpy(Addr.data(), reinterpret_cast<const sockaddr_in6*>(platform_data_.get())->sin6_addr.s6_addr,
            sizeof(reinterpret_cast<const sockaddr_in6*>(platform_data_.get())->sin6_addr.s6_addr));
        break;
    default:
        return RESULT_OPERATION_FAIL;
    }

    return RESULT_OK;
}

void SocketAddress::setLoopbackAddress(SocketAddress::AddressType Type)
{
    int prevPort = getPort();
    switch (Type)
    {
    case ADDRESS_TYPE_INET:
    {
        sockaddr_in* addrv4 = reinterpret_cast<sockaddr_in*>(platform_data_.get());
        addrv4->sin_family = AF_INET;
        addrv4->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        break;
    }
    case ADDRESS_TYPE_INET6:
    {
        sockaddr_in6* addrv6 = reinterpret_cast<sockaddr_in6*>(platform_data_.get());
        addrv6->sin6_family = AF_INET6;
        addrv6->sin6_addr = in6addr_loopback;
        break;
    }
    default:
        return;
    }

    setPort(prevPort);
}

void SocketAddress::setBroadcastAddressIPv4()
{
    int prevPort = getPort();
    sockaddr_in* addrv4 = reinterpret_cast<sockaddr_in*>(platform_data_.get());
    addrv4->sin_family = AF_INET;
    addrv4->sin_addr.s_addr = htonl(INADDR_BROADCAST);
    setPort(prevPort);
}

void SocketAddress::setAnyAddress(SocketAddress::AddressType Type)
{
    int prevPort = getPort();
    switch (Type)
    {
    case ADDRESS_TYPE_INET:
    {
        sockaddr_in* addrv4 = reinterpret_cast<sockaddr_in*>(platform_data_.get());
        addrv4->sin_family = AF_INET;
        addrv4->sin_addr.s_addr = htonl(INADDR_ANY);
        break;
    }
    case ADDRESS_TYPE_INET6:
    {
        sockaddr_in6* addrv6 = reinterpret_cast<sockaddr_in6*>(platform_data_.get());
        addrv6->sin6_family = AF_INET6;
        addrv6->sin6_addr = in6addr_any;
        break;
    }
    default:
        return;
    }

    setPort(prevPort);
}

const std::shared_ptr<sockaddr_storage> SocketAddress::getPlatformData() const
{
    return platform_data_;
}

size_t SocketAddress::LoopUpHostName(const std::string& HostName, const std::string& SeviceName, std::vector<SocketAddress>& AddressPool,
    bool PerformDNS, SocketAddress::AddressType Type)
{
    struct addrinfo hints;
    struct addrinfo* result;
    int ans;

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = halAddrTypeToOSType(Type);
    hints.ai_flags = AI_PASSIVE;

    if (!PerformDNS)
    {
        hints.ai_family |= AI_NUMERICSERV | AI_NUMERICHOST;
    }

    ans = getaddrinfo(HostName.c_str(), SeviceName.c_str(), &hints, &result);

    AddressPool.clear();

    if (ans != 0)
    {
        // hostname loopup failed
        return 0;
    }

    for (struct addrinfo* cursor = result; cursor != nullptr; cursor = cursor->ai_next)
    {
        if (cursor->ai_family == ADDRESS_TYPE_INET || cursor->ai_family == ADDRESS_TYPE_INET6)
        {
            auto storage = std::make_unique<sockaddr_storage>();
            assert(sizeof(sockaddr_storage) >= cursor->ai_addrlen);
            memcpy(storage.get(), cursor->ai_addr, cursor->ai_addrlen);
            AddressPool.push_back(SocketAddress(std::move(storage)));
        }
    }

    freeaddrinfo(result);

    return AddressPool.size();
}



StreamSocket::StreamSocket(SocketFamilyType Family/* = SOCKET_FAMILY_INET*/)
{
    socket_fd_ = ::socket(socketHalFamilyToOSFamily(Family), SOCK_STREAM, 0);
    assert(socket_fd_ >= 0);

    int boolTrue = 1;
    ::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, (char*)&boolTrue, sizeof(boolTrue));

    enableNoDelay(true);
    setTimeout(DEFAULT_SOCKET_TIMEOUT, SOCKET_DIR_BOTH);
}

StreamSocket::~StreamSocket()
{
    close(socket_fd_);
}

uint32_t StreamSocket::bind(const SocketAddress& LocalAddr)
{
    const struct sockaddr* addr = reinterpret_cast<const struct sockaddr*>(LocalAddr.getPlatformData().get());
    assert(addr);

    int ans = ::bind(socket_fd_, addr, sizeof(sockaddr_storage));
    if (ans)
    {
        return RESULT_OPERATION_FAIL;
    }
    else
    {
        return RESULT_OK;
    }
}

uint32_t StreamSocket::getLocalAddress(SocketAddress& Localaddr)
{
    struct sockaddr* addr = reinterpret_cast<struct sockaddr*>(Localaddr.getPlatformData().get());
    assert(addr);

    size_t actualsize = sizeof(sockaddr_storage);
    int ans = ::getsockname(socket_fd_, addr, (socklen_t*)&actualsize);

    assert(actualsize <= sizeof(sockaddr_storage));
    assert(addr->sa_family == AF_INET || addr->sa_family == AF_INET6);

    return ans ? RESULT_OPERATION_FAIL : RESULT_OK;
}

uint32_t StreamSocket::setTimeout(uint32_t Timeout, SocketBase::SocketDirectionMask Mask)
{
    timeval tv;
    tv.tv_sec = Timeout / 1000;
    tv.tv_usec = (Timeout % 1000) * 1000;

    if (Mask & SOCKET_DIR_RD)
    {
        int ans = ::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        if (ans)
        {
            return RESULT_OPERATION_FAIL;
        }
    }

    if (Mask & SOCKET_DIR_WR)
    {
        int ans = ::setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        if (ans)
        {
            return RESULT_OPERATION_FAIL;
        }
    }

    return RESULT_OK;
}

void delay(uint32_t Ms)
{
    while (Ms >= 1000)
    {
        usleep(1000 * 1000);
        Ms -= 1000;
    };
    if (Ms != 0)
    {
        usleep(Ms * 1000);
    }
}

uint32_t StreamSocket::waitforSent(uint32_t Timeout)
{
    fd_set wrset;
    FD_ZERO(&wrset);
    FD_SET(socket_fd_, &wrset);

    timeval tv;
    tv.tv_sec = Timeout / 1000;
    tv.tv_usec = (Timeout % 1000) * 1000;
    int ans = ::select(socket_fd_ + 1, nullptr, &wrset, nullptr, &tv);

    switch (ans)
    {
    case 1:
        // fired
        return RESULT_OK;
    case 0:
        // timeout
        return RESULT_OPERATION_TIMEOUT;
    default:
        delay(0); //relax cpu
        return RESULT_OPERATION_FAIL;
    }
}

uint32_t StreamSocket::waitforData(uint32_t Timeout)
{
    fd_set rdset;
    FD_ZERO(&rdset);
    FD_SET(socket_fd_, &rdset);

    timeval tv;
    tv.tv_sec = Timeout / 1000;
    tv.tv_usec = (Timeout % 1000) * 1000;
    int ans = ::select(socket_fd_ + 1, &rdset, nullptr, nullptr, &tv);

    switch (ans)
    {
    case 1:
        // fired
        return RESULT_OK;
    case 0:
        // timeout
        return RESULT_OPERATION_TIMEOUT;
    default:
        delay(0); //relax cpu
        return RESULT_OPERATION_FAIL;
    }
}

uint32_t StreamSocket::connect(const SocketAddress& AddrPack)
{
    const struct sockaddr* addr = reinterpret_cast<const struct sockaddr*>(AddrPack.getPlatformData().get());
    int ans = ::connect(socket_fd_, addr, sizeof(sockaddr_storage));
    if (!ans)
    {
        return RESULT_OK;
    }

    switch (errno)
    {
    case EAFNOSUPPORT:
        return RESULT_OPERATION_NOT_SUPPORT;
    case ETIMEDOUT:
        return RESULT_OPERATION_TIMEOUT;
    default:
        return RESULT_OPERATION_FAIL;
    }
}

uint32_t StreamSocket::listen(int Backlog)
{
    return ::listen(socket_fd_, Backlog) ? RESULT_OPERATION_FAIL : RESULT_OK;
}

std::shared_ptr<StreamSocket> StreamSocket::accept(std::shared_ptr<SocketAddress> AddrPack/* = nullptr*/)
{
    size_t addrsize = sizeof(sockaddr_storage);
    int pairSocket = ::accept(socket_fd_, AddrPack ?
        reinterpret_cast<struct sockaddr*>(AddrPack->getPlatformData().get()) : nullptr
        , (socklen_t*)&addrsize);

    //if (pairSocket >= 0)
    //{
    //    return std::make_shared<StreamSocket>(pairSocket);
    //}
    //else
    {
        return nullptr;
    }
}

uint32_t StreamSocket::waitforIncomingConnection(uint32_t Timeout)
{
    return waitforData(Timeout);
}

uint32_t StreamSocket::send(const void* Buffer, size_t Len)
{
    ssize_t ans = ::send(socket_fd_, Buffer, Len, MSG_NOSIGNAL);
    if (ans == (ssize_t)Len)
    {
        return RESULT_OK;
    }
    else
    {
        switch (errno)
        {
        case EAGAIN:
#if EWOULDBLOCK!=EAGAIN
        case EWOULDBLOCK:
#endif
            return RESULT_OPERATION_TIMEOUT;
        default:
            return RESULT_OPERATION_FAIL;
        }
    }
}

uint32_t StreamSocket::recv(void* Buffer, size_t Len, size_t& RecvLen)
{
    ssize_t ans = ::recv(socket_fd_, Buffer, Len, 0);
    if (ans == -1)
    {
        RecvLen = 0;

        switch (errno)
        {
        case EAGAIN:
#if EWOULDBLOCK!=EAGAIN
        case EWOULDBLOCK:
#endif
            return RESULT_OPERATION_TIMEOUT;
        default:
            return RESULT_OPERATION_FAIL;
        }
    }
    else
    {
        RecvLen = (size_t)ans;
        return RESULT_OK;
    }
}

uint32_t StreamSocket::getPeerAddress(SocketAddress& PeerAddr)
{
    struct sockaddr* addr = reinterpret_cast<struct sockaddr*>(PeerAddr.getPlatformData().get());
    assert(addr);

    size_t actualSize = sizeof(sockaddr_storage);
    int ans = ::getpeername(socket_fd_, addr, (socklen_t*)&actualSize);

    assert(actualsize <= sizeof(sockaddr_storage));
    assert(addr->sa_family == AF_INET || addr->sa_family == AF_INET6);

    return ans ? RESULT_OPERATION_FAIL : RESULT_OK;
}

uint32_t StreamSocket::shutdown(SocketBase::SocketDirectionMask Mask)
{
    int shutdwOpt;

    switch (Mask)
    {
    case SOCKET_DIR_RD:
        shutdwOpt = SHUT_RD;
        break;
    case SOCKET_DIR_WR:
        shutdwOpt = SHUT_WR;
        break;
    case SOCKET_DIR_BOTH:
    default:
        shutdwOpt = SHUT_RDWR;
    }

    return ::shutdown(socket_fd_, shutdwOpt) ? RESULT_OPERATION_FAIL : RESULT_OK;
}

uint32_t StreamSocket::enableKeepAlive(bool Enable)
{
    int boolTrue = Enable ? 1 : 0;
    return ::setsockopt(socket_fd_, SOL_SOCKET, SO_KEEPALIVE, &boolTrue, sizeof(boolTrue)) ?
        RESULT_OPERATION_FAIL : RESULT_OK;
}

uint32_t StreamSocket::enableNoDelay(bool Enable)
{
    int boolTrue = Enable ? 1 : 0;
    return ::setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &boolTrue, sizeof(boolTrue)) ?
        RESULT_OPERATION_FAIL : RESULT_OK;
}
