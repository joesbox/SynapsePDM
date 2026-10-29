#ifndef CORTEX_PORT_H
#define CORTEX_PORT_H

#include <Arduino.h>
#include "PinLock.h"

class CortexPort : public Stream
{
public:
    static constexpr uint8_t Control = 'a';
    static constexpr uint8_t Envelope = '!';
    static constexpr size_t MaxPayload = 6656;
    CortexPort(Stream &wire, PinLock &lock, void (*feedWatchdog)());
    void Poll(uint32_t now);
    bool NextCommand(uint32_t now);
    void FinishCommand();
    void Reset();
    bool ConsumeLockEvent();
    bool HasSession() const { return session; }
    bool InPacket() const { return packetCommand; }
    uint32_t LastActivity() const { return lastActivity; }
    int available() override;
    int read() override;
    int peek() override;
    void flush() override;
    using Print::write;
    size_t write(uint8_t value) override;
    size_t write(const uint8_t *data, size_t length) override;
    bool WritePlaintextLogPacket(const uint8_t *data, size_t length);

private:
    Stream &wire;
    PinLock &lock;
    void (*feedWatchdog)();
    bool session = false;
    bool lockEvent = false;
    bool packetCommand = false;
    bool rawCommand = false;
    bool publicCommand = false;
    uint8_t receiveKey[32] = {};
    uint8_t transmitKey[32] = {};
    uint8_t sessionID[8] = {};
    uint32_t receiveSequence = 0;
    uint32_t transmitSequence = 0;
    uint32_t lastActivity = 0;
    uint8_t header[7] = {};
    size_t headerUsed = 0;
    size_t bodyUsed = 0;
    size_t bodyLength = 0;
    uint32_t packetStarted = 0;
    uint32_t lastHello = 0;
    bool helloSeen = false;
    uint8_t input[MaxPayload + 16] = {};
    size_t inputPosition = 0;
    size_t inputLength = 0;
    PinLock::Operation pendingOperation = PinLock::Operation::Info;

    void ClearInput();
    bool WriteWire(const uint8_t *data, size_t length);
    bool SendEncrypted(const uint8_t *data, size_t length);
    void ControlReply(uint8_t operation, PinLock::Result result, const uint8_t *data, size_t length);
    void OperationReply(PinLock::Operation operation, PinLock::Result result,
                        const char recovery[PinLock::RecoveryLength], uint32_t now);
    bool ProcessPacket(uint32_t now);
    void ProcessControl(uint32_t now);
};

#endif
