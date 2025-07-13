#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#if defined(_WIN32)
// FIXME
#else
#include <sys/socket.h> // sockaddr, socklen_t
#endif

class SocketAddress final
{
public:
    SocketAddress();
    SocketAddress(const char *addr, uint16_t port);

    ~SocketAddress();

    sockaddr *getAddr();
    const sockaddr *getAddr() const;

    uint16_t getPort() const;

    std::string toString(bool withPort = false);

private:
    sockaddr *sinAddr = nullptr;
};

enum class SocketType
{
    TCP,
    UDP
};

class Socket final
{
public:
    Socket(SocketType type);
    Socket(SocketType type, int fd);
    Socket(Socket &) = delete;
    Socket(Socket &&other);

    ~Socket();

    Socket &operator=(Socket &&other);

    bool connect(const char *addr, uint16_t port, uint16_t sourcePort = 0);
    bool bind(const char *addr, uint16_t port);
    bool listen(const char *addr, uint16_t port);

    int recv(void *data, size_t len, int flags = 0);
    int recv(void *data, size_t len, SocketAddress *addr, int flags = 0);

    bool send(const void *data, size_t &len, int flags = 0);
    bool send(const void *data, size_t &len, const SocketAddress *addr, int flags = 0);
    bool sendAll(const void *data, size_t &len, int flags = 0);

    std::optional<Socket> accept(SocketAddress *addr = nullptr);

    int close();

    int getFd() const;

private:
    int getSockType() const;
    int getLastError();

    SocketType type;
    int fd = -1;
};