#include <cstring>
#include <iostream>

#if defined(_WIN32)
#include <Ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "Socket.h"

SocketAddress::SocketAddress()
{
    sinAddr = reinterpret_cast<sockaddr *>(new sockaddr_storage());
}

SocketAddress::SocketAddress(const char *addr, uint16_t port) : SocketAddress()
{
    auto sin6Addr = reinterpret_cast<sockaddr_in6 *>(sinAddr);
    sin6Addr->sin6_family = AF_INET6;
    sin6Addr->sin6_port = htons(port);

    if(inet_pton(AF_INET6, addr, &sin6Addr->sin6_addr) != 1)
    {
        delete sinAddr;
        sinAddr = nullptr;
    }
}

SocketAddress::~SocketAddress()
{
    delete reinterpret_cast<sockaddr_storage *>(sinAddr);
}

sockaddr *SocketAddress::getAddr()
{
    return sinAddr;
}

const sockaddr *SocketAddress::getAddr() const
{
    return sinAddr;
}

uint16_t SocketAddress::getPort() const
{
    if(!sinAddr)
        return 0;

    return reinterpret_cast<sockaddr_in *>(sinAddr)->sin_port;
}

std::string SocketAddress::toString(bool withPort)
{
    if(!sinAddr)
        return "";

    char ip[INET6_ADDRSTRLEN];
    const void *address;

    if(sinAddr->sa_family == AF_INET6)
        address = &reinterpret_cast<const struct sockaddr_in6 *>(sinAddr)->sin6_addr;
    else
        address = &reinterpret_cast<const struct sockaddr_in *>(sinAddr)->sin_addr;

    if(!inet_ntop(sinAddr->sa_family, address, ip, INET6_ADDRSTRLEN))
        return "";

    // add the port if requested
    if(withPort)
    {
        auto portStr = std::to_string(getPort());
        if(sinAddr->sa_family == AF_INET6)
        {
            // [::1:2:3:4]:5678
            std::string ret;
            ret.reserve(strlen(ip) + 3 + portStr.length());

            ret.append("[").append(ip).append("]:").append(portStr);

            return ret;
        }
        else
        {
            // 1.2.3.4:5678
            std::string ret;
            ret.reserve(strlen(ip) + 1 + portStr.length());

            ret.append(ip).append(":").append(portStr);

            return ret;
        }
    }

    return ip;
}

Socket::Socket(SocketType type) : type(type)
{
}

Socket::Socket(SocketType type, int fd) : type(type), fd(fd)
{
}

Socket::Socket(Socket &&other) : fd(-1)
{
    *this = std::move(other);
}

Socket::~Socket()
{
    close();
}

Socket &Socket::operator=(Socket &&other)
{
    if(this != &other)
    {
        if(fd != -1)
            close();

        type = other.type;
        fd = other.fd;

        other.fd = -1;
    }

    return *this;
}

bool Socket::connect(const char *addr, uint16_t port, uint16_t sourcePort)
{
    if(fd != -1)
        return false;

    // lookup address and try to connect
    auto portStr = std::to_string(port);

    struct addrinfo hints = {}, *res, *p;
    int status;

    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = getSockType();

    if((status = getaddrinfo(addr, portStr.c_str(), &hints, &res)) != 0)
        return false;

    for(p = res; p != nullptr; p = p->ai_next)
    {
        fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if(fd == -1)
            continue;

        if(sourcePort != 0)
        {
            // bind if we want a specific source port
            struct sockaddr_in6 sinAddr = {};
            sinAddr.sin6_family = AF_INET6;
            sinAddr.sin6_port = htons(sourcePort);
            sinAddr.sin6_addr = in6addr_any;

            if(::bind(fd, (struct sockaddr *)&sinAddr, sizeof(sinAddr)) == -1)
            {
                close();
                fd = -1;
                continue;
            }
        }

        if(::connect(fd, p->ai_addr, p->ai_addrlen) == -1)
        {
            // TODO: if EINPROGRESS, non-blocking connect
            close();
            fd = -1;
            continue;
        }

        break;
    }

    freeaddrinfo(res);

    return fd != -1;
}

bool Socket::bind(const char *addr, uint16_t port)
{
    if(fd != -1)
        return false;

    fd = socket(AF_INET6, getSockType(), 0);

    if(fd == -1)
        return false;

    int yes = 1;

    if(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char *>(&yes), sizeof(int)) == -1)
    {
        close();
        return false;
    }

    // allow IPv4 connections
    yes = 0; // no
    if(setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, reinterpret_cast<char *>(&yes), sizeof(int)) == -1)
    {
        close();
        return false;
    }

    // bind
    struct sockaddr_in6 sinAddr = {};
    sinAddr.sin6_family = AF_INET6;
    sinAddr.sin6_port = htons(port);

    if(inet_pton(AF_INET6, addr, &sinAddr.sin6_addr) != 1)
        return false;

    if(::bind(fd, (struct sockaddr *)&sinAddr, sizeof(sinAddr)) == -1)
    {
        close();
        return false;
    }
    return true;
}

bool Socket::listen(const char *addr, uint16_t port)
{
    // you don't listen on a UDP socket
    if(type == SocketType::UDP)
        return false;

    if(!bind(addr, port))
        return false;

    if(::listen(fd, 1) == -1)
    {
        close();
        return false;
    }

    return true;
}

int Socket::recv(void *data, size_t len, int flags)
{
    return recv(data, len, nullptr, flags);
}

int Socket::recv(void *data, size_t len, SocketAddress *addr, int flags)
{
    auto sockAddr = addr ? addr->getAddr() : nullptr;
    socklen_t addrLen = sizeof(sockaddr_storage);

    int ret = ::recvfrom(fd, reinterpret_cast<char *>(data), len, flags, sockAddr, sockAddr ? &addrLen : nullptr);

    if(ret == 0)
    {
        // disconnected
        close();
        fd = -1;
    }
    else if(ret == -1)
    {
        // possibly error, or maybe just a non-blocking socket
        auto err = getLastError();
        if(err == EWOULDBLOCK)
            return 0;
    }

    return ret;
}

bool Socket::send(const void *data, size_t &len, int flags)
{
    return send(data, len, nullptr, flags);
}

bool Socket::send(const void *data, size_t &len, const SocketAddress *addr, int flags)
{
    auto sockAddr = addr ? addr->getAddr() : nullptr;
    socklen_t addrLen = sizeof(sockaddr_storage);

    auto sent = ::sendto(fd, reinterpret_cast<const char *>(data), len, flags, sockAddr, addrLen);

    if(sent < 0)
        return false;

    len = sent;

    return true;
}

bool Socket::sendAll(const void *data, size_t &len, int flags)
{
    // doesn't make much sense on a UDP socket
    if(type != SocketType::TCP)
        return false;

    size_t total_sent = 0;
    size_t to_send = len;
    int sent = 0;

    while(to_send)
    {
        sent = ::send(fd, reinterpret_cast<const char *>(data) + total_sent, to_send, flags);
        if(sent == -1)
            break;
        total_sent += sent;
        to_send -= sent;
    }

    len = total_sent;
    return sent != -1;
}

std::optional<Socket> Socket::accept(SocketAddress *addr)
{
    if(type != SocketType::TCP)
        return {};

    auto sockAddr = addr ? addr->getAddr() : nullptr;
    socklen_t addrLen = sizeof(sockaddr_storage);

    int newFd = ::accept(fd, sockAddr, sockAddr ? &addrLen : nullptr);

    if(newFd == -1)
        return {};

    return Socket(type, newFd);
}

int Socket::close()
{
#ifdef _WIN32
    int ret = closesocket(fd);
#else
    int ret = ::close(fd);
#endif

    fd = -1;

    return ret;
}

int Socket::getFd() const
{
    return fd;
}

int Socket::getSockType() const
{
    switch(type)
    {
        case SocketType::TCP:
            return SOCK_STREAM;
        case SocketType::UDP:
            return SOCK_DGRAM;
    }

    return 0;
}

int Socket::getLastError()
{
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}