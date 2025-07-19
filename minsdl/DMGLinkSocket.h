#pragma once
#include <condition_variable>
#include <mutex>
#include <thread>

#include "Socket.h"

class DMGCPU;

class DMGLinkSocket final
{
public:
    DMGLinkSocket(DMGCPU &cpu);
    ~DMGLinkSocket();

    bool listen();
    bool connect(const char *addr);

    void update();

    void setLocalState(uint8_t sb, uint8_t sc, uint32_t cycle);
    uint8_t sync(uint32_t cycle, bool isInternalClock);

private:
    bool isConnected() const;

    void threadRun();

    bool handlePacket(Socket &socket);

    void sendPacket(const uint8_t *buf, size_t len);

    std::thread thread;
    bool exitThread = false;

    DMGCPU &cpu;

    Socket socket;
    std::optional<Socket> clientSocket;
    bool listening = false;

    uint8_t localSB = 0, localSC = 0;

    uint8_t remoteSB, remoteSC;
    uint32_t remoteCycle;
    bool haveRemoteState = false;
    uint8_t syncSB;

    // sent transfer complete and waiting for response
    bool waitingForRemoteSync = false;

    // received transfer complete from the device driving the clock
    bool doRemoteSync = false;
    bool expectRemoteSync = false;
    int remoteSyncAttempts = 0;

    std::mutex mutex;
    std::condition_variable condVar;
};