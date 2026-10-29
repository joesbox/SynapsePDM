#include "CortexPort.h"
#include <ChaChaPoly.h>
#include <Crypto.h>
#include <limits.h>
#include <string.h>

namespace
{
constexpr uint32_t TransmitTimeoutMs = 250;

uint32_t Load32(const uint8_t *data)
{
    return uint32_t(data[0]) | (uint32_t(data[1]) << 8) |
           (uint32_t(data[2]) << 16) | (uint32_t(data[3]) << 24);
}

void Store32(uint8_t *data, uint32_t value)
{
    for (uint8_t i = 0; i < 4; ++i) data[i] = value >> (8 * i);
}
}

CortexPort::CortexPort(Stream &port, PinLock &pinLock, void (*watchdog)())
    : wire(port), lock(pinLock), feedWatchdog(watchdog)
{
}

void CortexPort::ClearInput()
{
    clean(input);
    clean(header);
    headerUsed = bodyUsed = bodyLength = inputPosition = inputLength = 0;
    packetCommand = rawCommand = publicCommand = false;
}

void CortexPort::Reset()
{
    lockEvent |= session;
    session = false;
    lock.Lock();
    clean(receiveKey);
    clean(transmitKey);
    clean(sessionID);
    receiveSequence = transmitSequence = 0;
    ClearInput();
}

bool CortexPort::ConsumeLockEvent()
{
    bool event = lockEvent;
    lockEvent = false;
    return event;
}

void CortexPort::Poll(uint32_t now)
{
    if (session && uint32_t(now - lastActivity) >= 5000) Reset();
    if (headerUsed && uint32_t(now - packetStarted) >= 1000)
    {
        Reset();
        ControlReply(0xFF, PinLock::Result::Invalid, nullptr, 0);
    }
    if (lock.Busy())
    {
        lastActivity = now;
        PinLock::Result result;
        char recovery[PinLock::RecoveryLength];
        if (lock.Tick(now, result, recovery))
            OperationReply(pendingOperation, result, recovery, now);
        clean(recovery);
    }
}

bool CortexPort::WriteWire(const uint8_t *data, size_t length)
{
    uint32_t started = millis();
    while (length)
    {
        size_t count = wire.write(data, length);
        if (count > length) count = length;
        data += count;
        length -= count;
        if (length && uint32_t(millis() - started) >= TransmitTimeoutMs)
        {
            Reset();
            return false;
        }
        feedWatchdog();
    }
    return true;
}

void CortexPort::ControlReply(uint8_t operation, PinLock::Result result,
                              const uint8_t *data, size_t length)
{
    uint8_t reply[] = {Control, operation, static_cast<uint8_t>(result),
                       static_cast<uint8_t>(length), static_cast<uint8_t>(length >> 8)};
    if (WriteWire(reply, sizeof(reply)) && length) WriteWire(data, length);
}

bool CortexPort::SendEncrypted(const uint8_t *data, size_t length)
{
    if (!session || length > MaxPayload || transmitSequence == UINT32_MAX)
    {
        Reset();
        return false;
    }
    uint8_t prefix[7] = {Envelope, static_cast<uint8_t>(length), static_cast<uint8_t>(length >> 8)};
    Store32(prefix + 3, transmitSequence++);
    uint8_t nonce[12];
    memcpy(nonce, sessionID, 8);
    memcpy(nonce + 8, prefix + 3, 4);
    ChaChaPoly cipher;
    cipher.setKey(transmitKey, sizeof(transmitKey));
    cipher.setIV(nonce, sizeof(nonce));
    cipher.addAuthData(prefix, sizeof(prefix));
    const uint32_t started = millis();
    bool ok = WriteWire(prefix, sizeof(prefix));
    uint8_t block[64];
    while (ok && length)
    {
        if (uint32_t(millis() - started) >= TransmitTimeoutMs)
        {
            Reset();
            ok = false;
            break;
        }
        size_t count = length > sizeof(block) ? sizeof(block) : length;
        cipher.encrypt(block, data, count);
        ok = WriteWire(block, count);
        data += count;
        length -= count;
    }
    uint8_t tag[16];
    cipher.computeTag(tag, sizeof(tag));
    if (ok && uint32_t(millis() - started) >= TransmitTimeoutMs)
    {
        Reset();
        ok = false;
    }
    if (ok) ok = WriteWire(tag, sizeof(tag));
    clean(block);
    clean(tag);
    return ok;
}

void CortexPort::OperationReply(PinLock::Operation operation, PinLock::Result result,
                                const char recovery[PinLock::RecoveryLength], uint32_t now)
{
    uint8_t reply[8 + PinLock::RecoveryLength] = {Control, static_cast<uint8_t>(operation),
                                               static_cast<uint8_t>(result)};
    Store32(reply + 3, lock.RetryAfter(now));
    reply[7] = (lock.Enabled() ? 1 : 0) | (lock.Authorized() ? 2 : 0) | (lock.Faulted() ? 4 : 0);
    size_t length = 8;
    if (result == PinLock::Result::OK &&
        (operation == PinLock::Operation::SetPIN || operation == PinLock::Operation::Recover ||
         operation == PinLock::Operation::RotateRecovery))
    {
        memcpy(reply + length, recovery, PinLock::RecoveryLength);
        length += PinLock::RecoveryLength;
    }
    SendEncrypted(reply, length);
    clean(reply);
}

void CortexPort::ProcessControl(uint32_t now)
{
    using Operation = PinLock::Operation;
    using Result = PinLock::Result;
    uint8_t operation = header[1];
    if (lock.Busy())
    {
        ControlReply(operation, Result::Busy, nullptr, 0);
        return;
    }
    if (operation == uint8_t(Operation::Info) && bodyLength == 0)
    {
        uint8_t info[42] = {1};
        info[1] = (lock.Enabled() ? 1 : 0) | (lock.Provisioned() ? 2 : 0) | (lock.Faulted() ? 4 : 0) | 8;
        Store32(info + 2, PinLock::KdfIterations);
        Store32(info + 6, lock.RetryAfter(now));
        lock.PublicKey(info + 10);
        ControlReply(operation, lock.Faulted() ? Result::StorageError : Result::OK, info, sizeof(info));
    }
    else if (operation == uint8_t(Operation::Provision) && bodyLength == 32)
    {
        Result result = lock.Provision(input);
        uint8_t publicKey[32];
        lock.PublicKey(publicKey);
        ControlReply(operation, result, publicKey, sizeof(publicKey));
    }
    else if (operation == uint8_t(Operation::Hello) && bodyLength == 32)
    {
        if (helloSeen && uint32_t(now - lastHello) < 1000)
        {
            uint8_t retry[4];
            Store32(retry, 1000 - uint32_t(now - lastHello));
            ControlReply(operation, Result::Throttled, retry, sizeof(retry));
            return;
        }
        lastHello = now;
        helloSeen = true;
        uint8_t clientPublic[32];
        memcpy(clientPublic, input, sizeof(clientPublic));
        Reset();
        uint8_t response[40];
        Result result = lock.OpenSession(clientPublic, response, receiveKey, transmitKey, response + 32);
        if (result == Result::OK)
        {
            memcpy(sessionID, response + 32, sizeof(sessionID));
            session = true;
            lastActivity = now;
        }
        ControlReply(operation, result, result == Result::OK ? response : nullptr,
                     result == Result::OK ? sizeof(response) : 0);
        clean(clientPublic);
    }
    else ControlReply(operation, Result::Invalid, nullptr, 0);
}

bool CortexPort::ProcessPacket(uint32_t now)
{
    if (header[0] == Control)
    {
        ProcessControl(now);
        ClearInput();
        return false;
    }
    if (!session || Load32(header + 3) != receiveSequence || receiveSequence == UINT32_MAX)
    {
        Reset();
        ControlReply(0xFF, PinLock::Result::Invalid, nullptr, 0);
        return false;
    }
    uint8_t nonce[12];
    memcpy(nonce, sessionID, 8);
    memcpy(nonce + 8, header + 3, 4);
    ChaChaPoly cipher;
    cipher.setKey(receiveKey, sizeof(receiveKey));
    cipher.setIV(nonce, sizeof(nonce));
    cipher.addAuthData(header, sizeof(header));
    const size_t length = bodyLength - 16;
    cipher.decrypt(input, input, length);
    if (!cipher.checkTag(input + length, 16))
    {
        Reset();
        ControlReply(0xFF, PinLock::Result::Invalid, nullptr, 0);
        return false;
    }
    ++receiveSequence;
    lastActivity = now;
    headerUsed = bodyUsed = bodyLength = 0;
    if (length >= 2 && input[0] == Control)
    {
        auto operation = static_cast<PinLock::Operation>(input[1]);
        char recovery[PinLock::RecoveryLength];
        PinLock::Result result = lock.Execute(operation, input + 2, length - 2, now, recovery);
        if (result == PinLock::Result::Pending) pendingOperation = operation;
        else OperationReply(operation, result, recovery, now);
        clean(recovery);
        ClearInput();
        if (result == PinLock::Result::OK &&
            (operation == PinLock::Operation::Lock || operation == PinLock::Operation::Disable))
            Reset();
        return false;
    }
    if (lock.Busy() || !lock.Authorized() || length == 0)
    {
        char recovery[PinLock::RecoveryLength] = {};
        OperationReply(PinLock::Operation::Status, lock.Busy() ? PinLock::Result::Busy : PinLock::Result::Locked,
                       recovery, now);
        ClearInput();
        return false;
    }
    packetCommand = true;
    inputPosition = 0;
    inputLength = length;
    return true;
}

bool CortexPort::NextCommand(uint32_t now)
{
    if (rawCommand || packetCommand) return true;
    for (size_t count = 0; count < 1024 && wire.available(); ++count)
    {
        if (headerUsed == 0)
        {
            uint8_t next = wire.peek();
            if (next != Control && next != Envelope)
            {
                if ((!lock.Enabled() && !session) || next == 'b' || next == 'k' || next == 'v' || next == 'd')
                {
                    rawCommand = true;
                    publicCommand = next == 'b' || next == 'k' || next == 'v' || next == 'd';
                    return true;
                }
                wire.read();
                const uint8_t rejected = 'f';
                WriteWire(&rejected, 1);
                continue;
            }
            packetStarted = now;
            header[headerUsed++] = wire.read();
            continue;
        }
        size_t headerLength = header[0] == Control ? 4 : 7;
        if (headerUsed < headerLength)
        {
            header[headerUsed++] = wire.read();
            if (headerUsed < headerLength) continue;
            size_t lengthOffset = header[0] == Control ? 2 : 1;
            size_t length = uint16_t(header[lengthOffset]) | (uint16_t(header[lengthOffset + 1]) << 8);
            if (length > (header[0] == Control ? 32 : MaxPayload))
            {
                Reset();
                ControlReply(0xFF, PinLock::Result::Invalid, nullptr, 0);
                return false;
            }
            bodyLength = length + (header[0] == Envelope ? 16 : 0);
            if (bodyLength == 0) return ProcessPacket(now);
            continue;
        }
        input[bodyUsed++] = wire.read();
        if (bodyUsed == bodyLength) return ProcessPacket(now);
    }
    return false;
}

void CortexPort::FinishCommand()
{
    if (packetCommand)
    {
        // Blocking commands such as provisioning must leave time for the next request.
        lastActivity = millis();
        ClearInput();
    }
    rawCommand = publicCommand = false;
}

int CortexPort::available()
{
    if (packetCommand) return inputLength - inputPosition;
    return rawCommand ? wire.available() : 0;
}

int CortexPort::read()
{
    if (packetCommand) return inputPosition < inputLength ? input[inputPosition++] : -1;
    return rawCommand ? wire.read() : -1;
}

int CortexPort::peek()
{
    if (packetCommand) return inputPosition < inputLength ? input[inputPosition] : -1;
    return rawCommand ? wire.peek() : -1;
}

void CortexPort::flush()
{
    wire.flush();
}

size_t CortexPort::write(uint8_t value)
{
    return write(&value, 1);
}

bool CortexPort::WritePlaintextLogPacket(const uint8_t *data, size_t length)
{
    if (!session || !lock.Authorized() || lock.Busy()) return false;
    return WriteWire(data, length);
}

size_t CortexPort::write(const uint8_t *data, size_t length)
{
    if (publicCommand || (!lock.Enabled() && !session)) return wire.write(data, length);
    if (!session || !lock.Authorized()) return 0;
    size_t written = 0;
    while (written < length)
    {
        size_t count = length - written;
        if (count > MaxPayload) count = MaxPayload;
        if (!SendEncrypted(data + written, count)) break;
        written += count;
    }
    return written;
}
