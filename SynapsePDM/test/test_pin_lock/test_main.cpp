#include "../../src/PinLock.h"
#include "../../src/CortexPort.h"
#include "../../src/EEPROMBlocks.h"
#include <ChaChaPoly.h>
#include <Crypto.h>
#include <Curve25519.h>
#include <SHA256.h>
#include <RNG.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using Result = PinLock::Result;
using Operation = PinLock::Operation;
using Bytes = std::vector<uint8_t>;
#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); std::exit(1); } } while (0)

static uint8_t eeprom[2][PinLock::RecordSize];
static bool writeFailure;
static unsigned writes;
static uint32_t clockMs;
static uint32_t clockUs;
static uint32_t clockStepUs;
static uint8_t seed[32];
static char recovery[PinLock::RecoveryLength];

uint32_t millis() { return clockMs; }
uint32_t micros()
{
    uint32_t value = clockUs;
    clockUs += clockStepUs;
    return value;
}
void Watchdog() {}

// Curve25519 also links its unused dh1() helper. Any accidental platform RNG use must fail.
RNGClass::RNGClass() {}
RNGClass::~RNGClass() {}
void RNGClass::rand(uint8_t *, size_t) { CHECK(false); }
RNGClass RNG;

bool Read(uint8_t slot, uint8_t *data, size_t length)
{
    CHECK(slot < 2 && length == PinLock::RecordSize);
    memcpy(data, eeprom[slot], length);
    return true;
}

bool Write(uint8_t slot, const uint8_t *data, size_t length)
{
    CHECK(slot < 2 && length == PinLock::RecordSize);
    ++writes;
    if (writeFailure) return false;
    memcpy(eeprom[slot], data, length);
    return true;
}

void ResetFixture()
{
    memset(eeprom, 0xFF, sizeof(eeprom));
    writeFailure = false;
    writes = 0;
    clockMs = 0;
    clockUs = 0;
    clockStepUs = 25;
    for (uint8_t i = 0; i < 32; ++i) seed[i] = i + 1;
    memset(recovery, 0, sizeof(recovery));
}

Result Finish(PinLock &lock, uint32_t now)
{
    Result result = Result::Pending;
    unsigned passes = 0;
    while (!lock.Tick(now, result, recovery)) CHECK(++passes < 1000);
    CHECK(passes > 0);
    return result;
}

void Enable(PinLock &lock)
{
    lock.Begin(0);
    CHECK(lock.Provision(seed) == Result::OK);
    CHECK(lock.Execute(Operation::SetPIN, reinterpret_cast<const uint8_t *>("0123"), 4, 0, recovery) == Result::Pending);
    CHECK(Finish(lock, 0) == Result::OK);
    CHECK(lock.Enabled() && lock.Authorized());
}

Result Unlock(PinLock &lock, const char *pin, uint32_t now)
{
    return lock.Execute(Operation::Unlock, reinterpret_cast<const uint8_t *>(pin), strlen(pin), now, recovery);
}

Bytes Unhex(const char *text)
{
    Bytes result;
    while (*text)
    {
        unsigned value = 0;
        CHECK(std::sscanf(text, "%2x", &value) == 1);
        result.push_back(static_cast<uint8_t>(value));
        text += 2;
    }
    return result;
}

void test_defaults_and_known_pbkdf2_vector()
{
    PinLock lock(Read, Write);
    lock.Begin(0);
    CHECK(!lock.Enabled() && lock.Authorized() && !lock.Provisioned());
    uint8_t weak[32] = {};
    CHECK(lock.Provision(weak) == Result::Invalid);
    CHECK(writes == 0);
    Enable(lock);
    const Bytes salt = Unhex("1b1389a36afa5cd7170497c89b3b7b4b");
    const Bytes hash = Unhex("c3f398b7edd4f86f863a7af3731aac6de60a4ecbe31836afc755a0527a8ecef2");
    CHECK(memcmp(eeprom[1] + 48, salt.data(), salt.size()) == 0);
    CHECK(memcmp(eeprom[1] + 64, hash.data(), hash.size()) == 0);
    CHECK(memcmp(recovery, "OXGD5MU5UDB762B6AOO4EAEX6WBBT645", sizeof(recovery)) == 0);
    for (const auto &slot : eeprom)
    {
        CHECK(std::search(slot, slot + sizeof(slot), recovery, recovery + sizeof(recovery)) == slot + sizeof(slot));
        const uint8_t pin[] = {'0', '1', '2', '3'};
        CHECK(std::search(slot, slot + sizeof(slot), pin, pin + sizeof(pin)) == slot + sizeof(slot));
    }
    CHECK(lock.Provision(seed) == Result::Locked);
}

void test_pin_validation_and_authorization()
{
    PinLock lock(Read, Write);
    Enable(lock);
    lock.Lock();
    unsigned before = writes;
    CHECK(Unlock(lock, "123", 0) == Result::Invalid);
    CHECK(Unlock(lock, "12345", 0) == Result::Invalid);
    CHECK(Unlock(lock, "12a4", 0) == Result::Invalid);
    CHECK(writes == before);
    CHECK(lock.Execute(Operation::Disable, nullptr, 0, 0, recovery) == Result::Locked);
    CHECK(lock.Execute(Operation::SetPIN, reinterpret_cast<const uint8_t *>("9999"), 4, 0, recovery) == Result::Locked);
    CHECK(lock.Execute(Operation::RotateRecovery, nullptr, 0, 0, recovery) == Result::Locked);
    CHECK(Unlock(lock, "0123", 0) == Result::Pending);
    CHECK(!lock.Authorized());
    CHECK(Finish(lock, 0) == Result::OK);
    CHECK(lock.Authorized());
}

void test_kdf_time_budget_wraparound_and_stalled_clock()
{
    PinLock lock(Read, Write);
    Enable(lock);
    lock.Lock();
    CHECK(Unlock(lock, "0123", 0) == Result::Pending);
    const unsigned before = writes;
    clockUs = UINT32_MAX - 1000;
    clockStepUs = 250;
    const uint32_t started = clockUs;
    Result result = Result::Pending;
    CHECK(!lock.Tick(0, result, recovery));
    CHECK(uint32_t(clockUs - started) == 5250);
    CHECK(lock.Busy() && !lock.Authorized() && writes == before);

    // A stalled clock must still yield instead of completing the KDF in one pass.
    clockStepUs = 0;
    CHECK(!lock.Tick(0, result, recovery));
    CHECK(lock.Busy() && !lock.Authorized() && writes == before);
    unsigned remainingPasses = 1;
    while (!lock.Tick(0, result, recovery)) CHECK(++remainingPasses <= 38);
    CHECK(remainingPasses == 38 && result == Result::OK);
    CHECK(lock.Authorized());
}

void test_cached_kdf_matches_reference_after_pin_changes()
{
    PinLock lock(Read, Write);
    Enable(lock);
    for (const char *value : {"0000", "9876", "0123"})
    {
        const auto *key = reinterpret_cast<const uint8_t *>(value);
        const unsigned before = writes;
        CHECK(lock.Execute(Operation::SetPIN, key, 4, 0, recovery) == Result::Pending);
        CHECK(Finish(lock, 0) == Result::OK);
        CHECK(writes == before + 1);
        const uint8_t *record = eeprom[(writes - 1) % 2];
        uint8_t saltBlock[20];
        memcpy(saltBlock, record + 48, 16);
        memset(saltBlock + 16, 0, 3);
        saltBlock[19] = 1;
        uint8_t previous[32], expected[32];
        hmac<SHA256>(previous, sizeof(previous), key, 4, saltBlock, sizeof(saltBlock));
        memcpy(expected, previous, sizeof(expected));
        for (uint32_t round = 1; round < PinLock::KdfIterations; ++round)
        {
            uint8_t next[32];
            hmac<SHA256>(next, sizeof(next), key, 4, previous, sizeof(previous));
            memcpy(previous, next, sizeof(previous));
            for (size_t i = 0; i < sizeof(expected); ++i) expected[i] ^= next[i];
        }
        CHECK(memcmp(record + 64, expected, sizeof(expected)) == 0);
    }
}

void test_retries_survive_reset_and_abort()
{
    PinLock lock(Read, Write);
    Enable(lock);
    lock.Lock();
    CHECK(Unlock(lock, "9999", 100) == Result::Pending);
    CHECK(writes == 3);
    lock.Lock();
    lock.Begin(0);
    CHECK(lock.RetryAfter(0) == 1000);
    unsigned before = writes;
    CHECK(Unlock(lock, "0123", 999) == Result::Throttled);
    CHECK(writes == before);
    CHECK(Unlock(lock, "9999", 1000) == Result::Pending);
    CHECK(Finish(lock, 1000) == Result::Locked);
    CHECK(lock.RetryAfter(1000) == 1000);
    lock.Begin(0);
    CHECK(lock.RetryAfter(0) == 1000);
    CHECK(Unlock(lock, "0123", 1000) == Result::Pending);
    CHECK(Finish(lock, 1000) == Result::OK);
    lock.Begin(0);
    CHECK(lock.RetryAfter(0) == 0 && !lock.Authorized());
}

void test_delay_thresholds_and_wraparound()
{
    PinLock lock(Read, Write);
    Enable(lock);
    lock.Lock();
    uint32_t now = UINT32_MAX - 500;
    for (unsigned attempt = 1; attempt <= 11; ++attempt)
    {
        CHECK(Unlock(lock, "9999", now) == Result::Pending);
        CHECK(Finish(lock, now) == Result::Locked);
        uint32_t expected = attempt <= 3 ? 1000 : attempt <= 6 ? 30000 : attempt <= 9 ? 300000 : 3600000;
        CHECK(lock.RetryAfter(now) == expected);
        CHECK(lock.RetryAfter(now + expected - 1) == 1);
        now += expected;
        CHECK(lock.RetryAfter(now) == 0);
    }
}

void test_recovery_replaces_pin_and_revokes_old_key()
{
    PinLock lock(Read, Write);
    Enable(lock);
    char oldRecovery[sizeof(recovery)];
    memcpy(oldRecovery, recovery, sizeof(recovery));
    lock.Lock();
    uint8_t request[36];
    memcpy(request, oldRecovery, 32);
    memcpy(request + 32, "4567", 4);
    CHECK(lock.Execute(Operation::Recover, request, sizeof(request), 0, recovery) == Result::Pending);
    CHECK(Finish(lock, 0) == Result::OK);
    CHECK(memcmp(oldRecovery, recovery, sizeof(recovery)) != 0);
    lock.Lock();
    CHECK(lock.Execute(Operation::Recover, request, sizeof(request), 1000, recovery) == Result::Locked);
    CHECK(Unlock(lock, "4567", 2000) == Result::Pending);
    CHECK(Finish(lock, 2000) == Result::OK);
    lock.Lock();
    CHECK(Unlock(lock, "0123", 3000) == Result::Pending);
    CHECK(Finish(lock, 3000) == Result::Locked);
}

void test_rotation_and_disable()
{
    PinLock lock(Read, Write);
    Enable(lock);
    char previous[sizeof(recovery)];
    memcpy(previous, recovery, sizeof(recovery));
    CHECK(lock.Execute(Operation::RotateRecovery, nullptr, 0, 0, recovery) == Result::OK);
    CHECK(memcmp(previous, recovery, sizeof(recovery)) != 0);
    uint8_t publicKey[32], after[32];
    lock.PublicKey(publicKey);
    CHECK(lock.Execute(Operation::Disable, nullptr, 0, 0, recovery) == Result::OK);
    lock.Begin(0);
    CHECK(!lock.Enabled() && lock.Provisioned());
    lock.PublicKey(after);
    CHECK(memcmp(publicKey, after, sizeof(after)) == 0);
    for (size_t i = 48; i < 128; ++i) CHECK(eeprom[1][i] == 0);
}

void test_storage_errors_fail_closed()
{
    PinLock lock(Read, Write);
    Enable(lock);
    // Never fall back to the earlier, unlocked record after a torn/corrupt write.
    eeprom[1][100] ^= 1;
    lock.Begin(0);
    CHECK(lock.Faulted() && lock.Enabled() && !lock.Authorized());
    CHECK(Unlock(lock, "0123", 0) == Result::StorageError);
}

void test_unprovisioned_legacy_contents_are_reset()
{
    PinLock lock(Read, Write);
    memset(eeprom[0], 0x00, sizeof(eeprom[0]));
    memset(eeprom[1], 0x5A, sizeof(eeprom[1]));
    writeFailure = true;
    lock.Begin(0);
    CHECK(lock.Faulted() && lock.Enabled() && !lock.Authorized());
    writeFailure = false;
    lock.Begin(0);
    CHECK(!lock.Faulted() && !lock.Enabled() && lock.Authorized() && !lock.Provisioned());
    for (const auto &slot : eeprom)
        for (uint8_t value : slot) CHECK(value == 0xFF);
    Enable(lock);
    lock.Begin(0);
    CHECK(!lock.Faulted() && lock.Enabled() && !lock.Authorized());
}

void test_write_failure_cannot_authorize()
{
    PinLock lock(Read, Write);
    Enable(lock);
    lock.Lock();
    writeFailure = true;
    CHECK(Unlock(lock, "0123", 0) == Result::StorageError);
    CHECK(lock.Faulted() && !lock.Authorized());
}

struct Wire : Stream
{
    Bytes incoming, outgoing;
    size_t position = 0;
    size_t writeCalls = 0;
    bool blocked = false;
    int available() override { return static_cast<int>(incoming.size() - position); }
    int read() override { return available() ? incoming[position++] : -1; }
    int peek() override { return available() ? incoming[position] : -1; }
    void flush() override {}
    size_t write(uint8_t value) override { outgoing.push_back(value); return 1; }
    size_t write(const uint8_t *data, size_t length) override
    {
        ++writeCalls;
        if (blocked)
        {
            clockMs += 25;
            return 0;
        }
        outgoing.insert(outgoing.end(), data, data + length);
        return length;
    }
    void Send(const Bytes &value)
    {
        CHECK(available() == 0);
        incoming = value;
        position = 0;
        outgoing.clear();
    }
};

void Put32(uint8_t *data, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) data[i] = value >> (8 * i);
}

struct Client
{
    uint8_t rx[32], tx[32], id[8];
    uint32_t sequence = 0;
    void Hello(CortexPort &port, Wire &wire, uint32_t now = 0)
    {
        uint8_t secret[32], publicKey[32], shared[32];
        for (uint8_t i = 0; i < 32; ++i) secret[i] = 100 + i;
        secret[0] &= 248;
        secret[31] = (secret[31] & 127) | 64;
        CHECK(Curve25519::eval(publicKey, secret, nullptr));
        Bytes request = {'a', 2, 32, 0};
        request.insert(request.end(), publicKey, publicKey + 32);
        wire.Send(request);
        CHECK(!port.NextCommand(now));
        CHECK(port.HasSession());
        CHECK(wire.outgoing.size() == 45 && wire.outgoing[2] == uint8_t(Result::OK));
        CHECK(Curve25519::eval(shared, secret, wire.outgoing.data() + 5));
        memcpy(id, wire.outgoing.data() + 37, 8);
        const char *labels[] = {"SynapsePDM client v1", "SynapsePDM device v1"};
        uint8_t *keys[] = {tx, rx};
        uint8_t prk[32];
        hmac<SHA256>(prk, sizeof(prk), id, sizeof(id), shared, sizeof(shared));
        for (uint8_t i = 0; i < 2; ++i)
        {
            SHA256 hash;
            hash.resetHMAC(prk, sizeof(prk));
            hash.update(labels[i], strlen(labels[i]));
            hash.update(publicKey, 32);
            hash.update(wire.outgoing.data() + 5, 32);
            const uint8_t block = 1;
            hash.update(&block, 1);
            hash.finalizeHMAC(prk, sizeof(prk), keys[i], 32);
        }
        sequence = 0;
        wire.outgoing.clear();
    }
    Bytes Encrypt(const Bytes &plain)
    {
        Bytes message(7 + plain.size() + 16);
        message[0] = '!';
        message[1] = static_cast<uint8_t>(plain.size());
        message[2] = static_cast<uint8_t>(plain.size() >> 8);
        Put32(message.data() + 3, sequence++);
        uint8_t nonce[12];
        memcpy(nonce, id, 8);
        memcpy(nonce + 8, message.data() + 3, 4);
        ChaChaPoly cipher;
        CHECK(cipher.setKey(tx, 32));
        CHECK(cipher.setIV(nonce, 12));
        cipher.addAuthData(message.data(), 7);
        cipher.encrypt(message.data() + 7, plain.data(), plain.size());
        cipher.computeTag(message.data() + 7 + plain.size(), 16);
        return message;
    }
    Bytes Decrypt(const Bytes &message)
    {
        CHECK(message.size() >= 23 && message[0] == '!');
        size_t length = message[1] | (size_t(message[2]) << 8);
        CHECK(message.size() == length + 23);
        uint8_t nonce[12];
        memcpy(nonce, id, 8);
        memcpy(nonce + 8, message.data() + 3, 4);
        ChaChaPoly cipher;
        CHECK(cipher.setKey(rx, 32));
        CHECK(cipher.setIV(nonce, 12));
        cipher.addAuthData(message.data(), 7);
        Bytes plain(length);
        cipher.decrypt(plain.data(), message.data() + 7, length);
        CHECK(cipher.checkTag(message.data() + 7 + length, 16));
        return plain;
    }
    void Authenticate(CortexPort &port, Wire &wire, PinLock &lock)
    {
        wire.Send(Encrypt({'a', 3, '0', '1', '2', '3'}));
        CHECK(!port.NextCommand(0));
        CHECK(lock.Busy() && wire.outgoing.empty());
        for (unsigned i = 0; lock.Busy(); ++i)
        {
            CHECK(i < 1000);
            port.Poll(0);
        }
        Bytes reply = Decrypt(wire.outgoing);
        CHECK(reply.size() == 8 && reply[2] == uint8_t(Result::OK));
        wire.outgoing.clear();
    }
};

void test_transport_defaults_and_locked_allowlist()
{
    PinLock lock(Read, Write);
    lock.Begin(0);
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    wire.Send({'r'});
    CHECK(port.NextCommand(0) && port.read() == 'r');
    CHECK(port.write(uint8_t('c')) == 1 && wire.outgoing[0] == 'c');
    port.FinishCommand();
    Enable(lock);
    lock.Lock();
    for (unsigned command = 0; command < 256; ++command)
    {
        if (command == 'a' || command == '!' || command == 'b' || command == 'k' ||
            command == 'v' || command == 'd') continue;
        wire.Send({uint8_t(command)});
        CHECK(!port.NextCommand(0));
        CHECK(wire.outgoing == Bytes({'f'}));
        CHECK(port.available() == 0 && port.read() == -1);
    }
    for (uint8_t command : Bytes({'b', 'k', 'v', 'd'}))
    {
        wire.Send({command});
        CHECK(port.NextCommand(0) && port.read() == command);
        CHECK(port.write(uint8_t('c')) == 1);
        port.FinishCommand();
    }
}

void test_transport_unlock_and_plaintext_bypass_denied()
{
    PinLock lock(Read, Write);
    Enable(lock);
    lock.Lock();
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    Client client;
    client.Hello(port, wire);
    wire.Send(client.Encrypt({'S'}));
    CHECK(!port.NextCommand(0));
    CHECK(client.Decrypt(wire.outgoing)[2] == uint8_t(Result::Locked));
    client.Authenticate(port, wire, lock);
    wire.Send({'P'});
    CHECK(!port.NextCommand(0) && wire.outgoing == Bytes({'f'}));
    wire.Send(client.Encrypt({'P'}));
    CHECK(port.NextCommand(0) && port.InPacket() && port.read() == 'P');
    CHECK(port.available() == 0 && port.read() == -1);
    CHECK(port.write(uint8_t('c')) == 1);
    CHECK(client.Decrypt(wire.outgoing) == Bytes({'c'}));
    port.FinishCommand();
    CHECK(port.available() == 0);
}

void test_transport_replay_and_tamper_close_session()
{
    for (unsigned tamper = 0; tamper < 4; ++tamper)
    {
        ResetFixture();
        PinLock lock(Read, Write);
        Enable(lock);
        Wire wire;
        CortexPort port(wire, lock, Watchdog);
        Client client;
        client.Hello(port, wire);
        client.Authenticate(port, wire, lock);
        Bytes packet = client.Encrypt({'S'});
        if (tamper == 0) packet.back() ^= 1;
        else if (tamper == 1) packet[3] ^= 1;
        else if (tamper == 3) packet[7] ^= 1;
        else
        {
            wire.Send(packet);
            CHECK(port.NextCommand(0));
            port.read();
            port.FinishCommand();
        }
        wire.Send(packet);
        CHECK(!port.NextCommand(0));
        CHECK(!port.HasSession() && !lock.Authorized());
        CHECK(wire.outgoing.size() == 5 && wire.outgoing[2] == uint8_t(Result::Invalid));
    }
}

void test_transport_lock_timeout_and_fragmentation()
{
    PinLock lock(Read, Write);
    Enable(lock);
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    Client client;
    client.Hello(port, wire);
    client.Authenticate(port, wire, lock);
    Bytes packet = client.Encrypt({'n', 0x84, 0x19, 'P'});
    for (size_t i = 0; i < packet.size(); ++i)
    {
        wire.Send({packet[i]});
        CHECK(port.NextCommand(1) == (i + 1 == packet.size()));
    }
    CHECK(port.read() == 'n' && port.available() == 3);
    clockMs = 1;
    port.FinishCommand();
    CHECK(port.available() == 0);
    port.Poll(5000);
    CHECK(port.HasSession());
    port.Poll(5001);
    CHECK(!port.HasSession() && !lock.Authorized() && port.ConsumeLockEvent());
}

void test_long_command_completion_restarts_session_idle_timeout()
{
    for (bool pinEnabled : {false, true})
    {
        ResetFixture();
        PinLock lock(Read, Write);
        if (pinEnabled) Enable(lock);
        else
        {
            lock.Begin(0);
            CHECK(lock.Provision(seed) == Result::OK);
        }
        Wire wire;
        CortexPort port(wire, lock, Watchdog);
        Client client;
        client.Hello(port, wire);
        if (pinEnabled) client.Authenticate(port, wire, lock);
        wire.Send(client.Encrypt({'M'}));
        CHECK(port.NextCommand(0) && port.read() == 'M');

        clockMs = 180000;
        CHECK(port.write(uint8_t('c')) == 1);
        CHECK(client.Decrypt(wire.outgoing) == Bytes({'c'}));
        wire.outgoing.clear();
        port.FinishCommand();
        CHECK(port.LastActivity() == clockMs);
        port.Poll(clockMs);
        CHECK(port.HasSession() && lock.Authorized());

        wire.Send(client.Encrypt({'n', 1}));
        clockMs += 1;
        CHECK(port.NextCommand(clockMs) && port.read() == 'n' && port.read() == 1);
        port.FinishCommand();
        port.Poll(clockMs + 4999);
        CHECK(port.HasSession());
        port.Poll(clockMs + 5000);
        CHECK(!port.HasSession());
        CHECK(!pinEnabled || !lock.Authorized());
    }
}

void test_transport_explicit_lock_and_no_recovery_readback()
{
    PinLock lock(Read, Write);
    Enable(lock);
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    Client client;
    client.Hello(port, wire);
    client.Authenticate(port, wire, lock);
    wire.Send(client.Encrypt({'a', 9}));
    CHECK(!port.NextCommand(0));
    CHECK(client.Decrypt(wire.outgoing).size() == 8);
    wire.Send(client.Encrypt({'a', 8}));
    CHECK(!port.NextCommand(0));
    CHECK(client.Decrypt(wire.outgoing)[2] == uint8_t(Result::OK));
    CHECK(!port.HasSession() && !lock.Authorized());
}

void test_transport_malformed_headers_and_handshake_counter()
{
    PinLock lock(Read, Write);
    Enable(lock);
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    Client client;
    client.Hello(port, wire);
    uint8_t previousID[8];
    memcpy(previousID, client.id, 8);
    unsigned before = writes;
    client.Hello(port, wire, 1000);
    CHECK(memcmp(previousID, client.id, 8) != 0);
    CHECK(writes == before);
    wire.Send({'!', 0xFF, 0xFF, 0, 0, 0, 0});
    CHECK(!port.NextCommand(0) && !port.HasSession());
    CHECK(wire.outgoing[2] == uint8_t(Result::Invalid));
    wire.Send({'a', 0});
    CHECK(!port.NextCommand(0));
    port.Poll(1000);
    CHECK(wire.outgoing[2] == uint8_t(Result::Invalid));
}

void test_factory_erase_preserves_only_security_region()
{
    struct EEPROM
    {
        uint8_t bytes[8192];
        void EepromWrite(uint16_t address, uint8_t length, uint8_t *data)
        {
            CHECK(length && address + length <= sizeof(bytes));
            CHECK(address / 32 == (address + length - 1) / 32);
            memcpy(bytes + address, data, length);
            memset(data, 0, length);
        }
        void EepromRead(uint16_t address, uint8_t length, uint8_t *data)
        {
            memcpy(data, bytes + address, length);
        }
        void EepromWaitEndWriteOperation() {}
    } device;
    memset(device.bytes, 0xA5, sizeof(device.bytes));
    constexpr uint16_t securityStart = 3348;
    EraseEEPROMOutsideRange<32>(device, sizeof(device.bytes), securityStart, 2 * PinLock::RecordSize);
    for (size_t i = 0; i < sizeof(device.bytes); ++i)
    {
        bool preserved = i >= securityStart && i < securityStart + 2 * PinLock::RecordSize;
        CHECK(device.bytes[i] == (preserved ? 0xA5 : 0xFF));
    }
}

void test_transport_enrollment_rotation_and_disable()
{
    PinLock lock(Read, Write);
    lock.Begin(0);
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    Bytes provision = {'a', 1, 32, 0};
    provision.insert(provision.end(), seed, seed + sizeof(seed));
    wire.Send(provision);
    CHECK(!port.NextCommand(0));
    CHECK(lock.Provisioned() && !lock.Enabled() && wire.outgoing[2] == 0);
    Client client;
    client.Hello(port, wire);
    Bytes previousRecovery;
    const Bytes requests[] = {
        {'a', 4, '0', '1', '2', '3'},
        {'a', 4, '4', '5', '6', '7'},
        {'a', 6}
    };
    for (const Bytes &request : requests)
    {
        wire.Send(client.Encrypt(request));
        CHECK(!port.NextCommand(0));
        for (unsigned i = 0; lock.Busy(); ++i)
        {
            CHECK(i < 1000);
            port.Poll(0);
        }
        Bytes reply = client.Decrypt(wire.outgoing);
        CHECK(reply.size() == 40 && reply[2] == uint8_t(Result::OK));
        CHECK(lock.Enabled() && lock.Authorized());
        Bytes key(reply.begin() + 8, reply.end());
        CHECK(key != previousRecovery);
        for (uint8_t c : key) CHECK((c >= 'A' && c <= 'Z') || (c >= '2' && c <= '7'));
        previousRecovery = key;
    }
    wire.Send(client.Encrypt({'a', 7}));
    CHECK(!port.NextCommand(0));
    CHECK(client.Decrypt(wire.outgoing)[2] == uint8_t(Result::OK));
    CHECK(!lock.Enabled() && !port.HasSession());
    wire.Send({'S'});
    CHECK(port.NextCommand(0) && port.read() == 'S');
    port.FinishCommand();
}

void test_session_ids_survive_reboot_and_bad_points()
{
    PinLock lock(Read, Write);
    Enable(lock);
    uint8_t client[32] = {9};
    uint8_t publicKey[32], receiveKey[32], transmitKey[32], first[8], second[8];
    CHECK(lock.OpenSession(client, publicKey, receiveKey, transmitKey, first) == Result::OK);
    unsigned before = writes;
    CHECK(lock.OpenSession(client, publicKey, receiveKey, transmitKey, second) == Result::OK);
    CHECK(memcmp(first, second, 8) != 0 && writes == before);
    lock.Begin(0);
    CHECK(lock.OpenSession(client, publicKey, receiveKey, transmitKey, first) == Result::OK);
    CHECK(memcmp(first, second, 8) != 0 && writes == before + 1);
    uint8_t zero[32] = {};
    CHECK(lock.OpenSession(zero, publicKey, receiveKey, transmitKey, second) == Result::Invalid);
}

void test_failed_final_write_does_not_unlock()
{
    PinLock lock(Read, Write);
    Enable(lock);
    lock.Lock();
    CHECK(Unlock(lock, "0123", 0) == Result::Pending);
    writeFailure = true;
    CHECK(Finish(lock, 0) == Result::StorageError);
    CHECK(lock.Enabled() && lock.Faulted() && !lock.Authorized());
}

void test_handshake_throttle_and_maximum_packet()
{
    PinLock lock(Read, Write);
    Enable(lock);
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    Client client;
    client.Hello(port, wire);
    Bytes request = {'a', 2, 32, 0};
    request.resize(36, 9);
    unsigned before = writes;
    wire.Send(request);
    CHECK(!port.NextCommand(999));
    CHECK(wire.outgoing.size() == 9 && wire.outgoing[2] == uint8_t(Result::Throttled));
    CHECK(wire.outgoing[5] == 1 && writes == before && port.HasSession());
    client.Authenticate(port, wire, lock);
    Bytes plain(CortexPort::MaxPayload, 0xA5);
    plain[0] = 'M';
    wire.Send(client.Encrypt(plain));
    bool ready = false;
    for (unsigned i = 0; !ready; ++i)
    {
        CHECK(i < 10);
        ready = port.NextCommand(0);
    }
    CHECK(port.available() == static_cast<int>(plain.size()));
    for (uint8_t byte : plain) CHECK(port.read() == byte);
    CHECK(port.read() == -1);
    Bytes reply(CortexPort::MaxPayload + 1, 'Z');
    CHECK(port.write(reply.data(), reply.size()) == reply.size());
    size_t firstLength = CortexPort::MaxPayload + 23;
    CHECK(client.Decrypt(Bytes(wire.outgoing.begin(), wire.outgoing.begin() + firstLength)).size() == CortexPort::MaxPayload);
    CHECK(client.Decrypt(Bytes(wire.outgoing.begin() + firstLength, wire.outgoing.end())) == Bytes({'Z'}));
    port.FinishCommand();
}

void test_transmit_backpressure_closes_session()
{
    PinLock lock(Read, Write);
    Enable(lock);
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    Client client;
    client.Hello(port, wire);
    client.Authenticate(port, wire, lock);
    wire.blocked = true;
    uint32_t started = clockMs;
    CHECK(port.write(uint8_t('c')) == 0);
    CHECK(!port.HasSession() && !lock.Authorized());
    CHECK(uint32_t(clockMs - started) == 250);
}

void test_plaintext_logs_require_authorization_and_preserve_encryption()
{
    PinLock lock(Read, Write);
    Enable(lock);
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    Client client;
    const Bytes log = {'B', 'L', 100, 1, 0, 0, 0, 0, 0, 0};
    CHECK(!port.WritePlaintextLogPacket(log.data(), log.size()));
    wire.Send({'a', 0, 0, 0});
    CHECK(!port.NextCommand(0));
    CHECK(wire.outgoing.size() == 47 && (wire.outgoing[6] & 8));
    wire.outgoing.clear();
    client.Hello(port, wire);
    CHECK(!port.WritePlaintextLogPacket(log.data(), log.size()));
    wire.Send(client.Encrypt({'y', 1}));
    CHECK(!port.NextCommand(0));
    CHECK(client.Decrypt(wire.outgoing)[2] == uint8_t(Result::Locked));
    client.Authenticate(port, wire, lock);
    wire.Send(client.Encrypt({'y', 1}));
    CHECK(port.NextCommand(0) && port.InPacket());
    CHECK(port.read() == 'y' && port.available() == 1 && port.read() == 1);
    const size_t before = wire.writeCalls;
    CHECK(port.WritePlaintextLogPacket(log.data(), log.size()));
    CHECK(wire.outgoing == log);
    CHECK(wire.writeCalls == before + 1);
    port.FinishCommand();
    wire.outgoing.clear();
    CHECK(port.write(uint8_t('c')) == 1);
    CHECK(client.Decrypt(wire.outgoing) == Bytes({'c'}));
    CHECK(wire.outgoing[3] == 2);
    wire.outgoing.clear();
    wire.Send({'S', 'w', 'y'});
    CHECK(!port.NextCommand(0) && wire.outgoing == Bytes({'f', 'f', 'f'}));
    port.Poll(5000);
    wire.outgoing.clear();
    CHECK(!port.WritePlaintextLogPacket(log.data(), log.size()));
    CHECK(wire.outgoing.empty());
}

void test_plaintext_log_backpressure_closes_session()
{
    PinLock lock(Read, Write);
    Enable(lock);
    Wire wire;
    CortexPort port(wire, lock, Watchdog);
    Client client;
    client.Hello(port, wire);
    client.Authenticate(port, wire, lock);
    wire.blocked = true;
    const uint8_t log[] = {'B', 'L'};
    CHECK(!port.WritePlaintextLogPacket(log, sizeof(log)));
    CHECK(!port.HasSession() && !lock.Authorized() && clockMs == 250);
}

int main()
{
    using Test = void (*)();
    const Test tests[] = {
        test_defaults_and_known_pbkdf2_vector, test_pin_validation_and_authorization,
        test_kdf_time_budget_wraparound_and_stalled_clock,
        test_cached_kdf_matches_reference_after_pin_changes,
        test_retries_survive_reset_and_abort, test_delay_thresholds_and_wraparound,
        test_recovery_replaces_pin_and_revokes_old_key, test_rotation_and_disable,
        test_storage_errors_fail_closed, test_unprovisioned_legacy_contents_are_reset, test_write_failure_cannot_authorize,
        test_transport_defaults_and_locked_allowlist, test_transport_unlock_and_plaintext_bypass_denied,
        test_transport_replay_and_tamper_close_session, test_transport_lock_timeout_and_fragmentation,
        test_long_command_completion_restarts_session_idle_timeout,
        test_transport_explicit_lock_and_no_recovery_readback, test_transport_malformed_headers_and_handshake_counter,
        test_factory_erase_preserves_only_security_region, test_transport_enrollment_rotation_and_disable,
        test_session_ids_survive_reboot_and_bad_points, test_failed_final_write_does_not_unlock,
        test_handshake_throttle_and_maximum_packet, test_transmit_backpressure_closes_session,
        test_plaintext_logs_require_authorization_and_preserve_encryption,
        test_plaintext_log_backpressure_closes_session
    };
    for (Test test : tests)
    {
        ResetFixture();
        test();
    }
    std::printf("%zu PIN lock and encrypted transport tests passed\n", sizeof(tests) / sizeof(tests[0]));
}
