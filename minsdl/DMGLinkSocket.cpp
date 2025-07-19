#include <iostream>

#include "DMGLinkSocket.h"

#include "DMGCPU.h"

static const int port = 0x4742;

enum class NetPacketType : uint8_t
{
    SetRemoteState = 0, // sends SB/SC state
    IntTransferComplete, // transfer has completed, sent from side using internal clock
    ExtTransferComplete, // sent from the other side after processing above
};

DMGLinkSocket::DMGLinkSocket(DMGCPU &cpu) : cpu(cpu), socket(SocketType::TCP)
{
}

DMGLinkSocket::~DMGLinkSocket()
{
    exitThread = true;
    if(thread.joinable())
        thread.join();
}

bool DMGLinkSocket::listen()
{
    if(!socket.listen("::", port))
        return false;

    listening = true;

    // launch thread
    thread = std::thread([this](){threadRun();});

    return true;
}

bool DMGLinkSocket::connect(const char *addr)
{
    if(!socket.connect(addr, port))
        return false;

    // launch thread
    thread = std::thread([this](){threadRun();});

    return true;
}

void DMGLinkSocket::update()
{
}

void DMGLinkSocket::setLocalState(uint8_t sb, uint8_t sc, uint32_t cycle)
{
    std::lock_guard lock(mutex);

    if(!isConnected())
        return;

    uint8_t buf[7];

    buf[0] = uint8_t(NetPacketType::SetRemoteState);
    buf[1] = sb;
    buf[2] = sc;
    buf[3] = (cycle >> 24) & 0xFF;
    buf[4] = (cycle >> 16) & 0xFF;
    buf[5] = (cycle >>  8) & 0xFF;
    buf[6] = cycle & 0xFF;


    localSB = sb;
    localSC = sc;

    sendPacket(buf, sizeof(buf));
}

uint8_t DMGLinkSocket::sync(uint32_t cycle, bool isInternalClock)
{
    std::unique_lock lock(mutex);

    if(!isConnected())
        return 0xFF;

    if(!isInternalClock)
    {
        if(expectRemoteSync && remoteSyncAttempts >= 8)
        {
            // it has been 8 serial clocks since we were told to expect a transfer
            // wait a bit for the other side to catch up
            for(int i = 0; i < 10 && !doRemoteSync; i++)
                condVar.wait_for(lock, std::chrono::milliseconds(1));
        }

        if(doRemoteSync)
        {
            // pass SB to cpu (remote should be providing clock if we got here)

            cpu.doExternalLinkPortTransfer(syncSB);
            doRemoteSync = false;

            // inform the other side that we've synced
            uint8_t buf[5];
            buf[0] = uint8_t(NetPacketType::ExtTransferComplete);
            
            uint32_t cycle = cpu.getCycleCount();
            buf[1] = (cycle >> 24) & 0xFF;
            buf[2] = (cycle >> 16) & 0xFF;
            buf[3] = (cycle >>  8) & 0xFF;
            buf[4] = cycle & 0xFF;

            sendPacket(buf, sizeof(buf));
        }
        else if(expectRemoteSync)
            remoteSyncAttempts++;

        return 0; // unused, will call doExternalLinkPortTransfer if we have something
    }

    if(waitingForRemoteSync)
    {
        // the other end hasn't replied about the last completion
        // wait a bit in the hope that they're still there
        for(int i = 0; i < 50 && waitingForRemoteSync; i++)
            condVar.wait_for(lock, std::chrono::milliseconds(1));
    }

    uint8_t buf[5];
    buf[0] = uint8_t(NetPacketType::IntTransferComplete);
    buf[1] = (cycle >> 24) & 0xFF;
    buf[2] = (cycle >> 16) & 0xFF;
    buf[3] = (cycle >>  8) & 0xFF;
    buf[4] = cycle & 0xFF;

    sendPacket(buf, sizeof(buf));

    waitingForRemoteSync = true;

    uint8_t ret = 0;

    // wait for remote state
    for(int i = 0; i < 10 && !haveRemoteState; i++)
        condVar.wait_for(lock, std::chrono::milliseconds(1));


    if(haveRemoteState)
    {
        // if the other side is also using an internal clock this isn't going to work
        if(!(remoteSC & 1))
            ret = remoteSB;
    }
    // else stall?

    haveRemoteState = false;

    return ret;
}

bool DMGLinkSocket::isConnected() const
{
    if(socket.getFd() == -1)
        return false;

    if(listening && !clientSocket)
        return false;

    return true;
}

void DMGLinkSocket::threadRun()
{
    while(!exitThread)
    {
        if(socket.getFd() == -1)
            return;

        fd_set fds;
        int maxFd = -1;
        FD_ZERO(&fds);

        auto addFd = [&fds, &maxFd](int fd)
        {
            FD_SET(fd, &fds);

            if(fd > maxFd)
                maxFd = fd;
        };

        // only check for new connections if we don't have an active one (only 1-1 for now)
        // also check main socket if not listening
        if(clientSocket)
            addFd(clientSocket->getFd());
        else
            addFd(socket.getFd());

        timeval t;
        t.tv_sec = 0;
        t.tv_usec = 10000;
        int ready = select(maxFd + 1, &fds, nullptr, nullptr, &t);

        if(ready < 0)
        {} // ohno
        else if(ready == 0)
            continue;

        condVar.notify_all();

        if(FD_ISSET(socket.getFd(), &fds))
        {
            if(listening)
            {
                SocketAddress addr;
                clientSocket = socket.accept(&addr);

                if(clientSocket)
                {
                    std::cout << "LinkSocket: " << addr.toString(true) << " connected" << std::endl;
                }
            }
            else
            {
                std::lock_guard lock(mutex);
                // handle packets on connected socket
                handlePacket(socket);
            }
        }

        if(clientSocket && FD_ISSET(clientSocket->getFd(), &fds))
        {
            if(!handlePacket(*clientSocket))
                clientSocket.reset();
        }
    }
}

bool DMGLinkSocket::handlePacket(Socket &socket)
{
    NetPacketType packet;
    int len = socket.recv(&packet, 1);

    if(len == 0)
    {
        // disconnect
        socket.close();
        return false;
    }
    else if(len == 1)
    {
        uint8_t buf[6];
        switch(packet)
        {
            case NetPacketType::SetRemoteState:
            {
                socket.recv(buf, 6);

                remoteSB = buf[0];
                remoteSC = buf[1];
                remoteCycle = buf[2] << 24 | buf[3] << 16 | buf[4] << 8 | buf[5];
                // get cur local cycle?
                haveRemoteState = true;

                remoteSyncAttempts = 0;
                
                if((remoteSC & 0x81) == 0x81)
                    expectRemoteSync = true;

                break;
            }
            case NetPacketType::IntTransferComplete:
            {
                socket.recv(buf, 4);

                //remoteCycle = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
                syncSB = remoteSB;
                doRemoteSync = true;
                expectRemoteSync = false;

                haveRemoteState = false;

                break;
            }

            case NetPacketType::ExtTransferComplete:
            {
                socket.recv(buf, 4);
                waitingForRemoteSync = false;
                break;
            }
        }
    }
    return true;
}

void DMGLinkSocket::sendPacket(const uint8_t *buf, size_t len)
{
    if(listening)
        clientSocket->sendAll(buf, len);
    else
        socket.sendAll(buf, len);
}