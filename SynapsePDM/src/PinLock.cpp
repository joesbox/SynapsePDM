#include "PinLock.h"
#include <Arduino.h>
#include <Crypto.h>
#include <Curve25519.h>
#include <HKDF.h>
#include <SHA256.h>
#include <string.h>
#include <limits.h>

namespace
{
constexpr uint32_t Magic = 0x314E4950;

bool Blank(const void *data, size_t length)
{
    const auto *bytes = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < length; ++i)
        if (bytes[i] != 0xFF) return false;
    return true;
}

void Clamp(uint8_t key[32])
{
    key[0] &= 248;
    key[31] = (key[31] & 127) | 64;
}

void RecoveryDigest(const uint8_t salt[16], const uint8_t *key, uint8_t output[32])
{
    SHA256 hash;
    hash.update("SynapsePDM recovery v1", 22);
    hash.update(salt, 16);
    hash.update(key, PinLock::RecoveryLength);
    hash.finalize(output, 32);
}
}

PinLock::PinLock(ReadRecord read, WriteRecord write)
    : readRecord(read), writeRecord(write)
{
}

void PinLock::Digest(Record &value)
{
    SHA256 hash;
    hash.update(&value, offsetof(Record, Digest));
    hash.finalize(value.Digest, sizeof(value.Digest));
}

bool PinLock::Valid(const Record &value)
{
    Record copy = value;
    Digest(copy);
    bool valid = value.Magic == Magic && value.Version == 1 &&
                 value.Enabled <= 1 && value.Failures <= 10 && value.Reserved == 0 &&
                 value.Generation != 0 && value.Generation <= UINT32_MAX &&
                 secure_compare(value.Digest, copy.Digest, sizeof(value.Digest));
    clean(copy);
    return valid;
}

void PinLock::Begin(uint32_t now)
{
    Lock();
    fault = false;
    activeSlot = -1;
    sessionEpoch = sessionNumber = 0;
    clean(identity);
    clean(record);
    Record slots[2];
    bool readFailed = false;
    bool invalidFound = false;
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (!readRecord(i, reinterpret_cast<uint8_t *>(&slots[i]), sizeof(Record)))
            readFailed = true;
        else if (!Blank(&slots[i], sizeof(Record)))
        {
            if (!Valid(slots[i])) invalidFound = true;
            else if (activeSlot < 0 || slots[i].Generation > record.Generation)
            {
                record = slots[i];
                activeSlot = i;
            }
        }
    }
    clean(slots);
    if (readFailed)
        fault = true;
    else if (invalidFound)
    {
        // Saves alternate slots, so once any record has been committed one valid slot always remains.
        // An invalid slot beside a valid one may be a torn newer write and must never resurrect the
        // older state. With no valid slot the lock was never provisioned (legacy EEPROM contents or a
        // torn first write), so the region is reset rather than bricking serial access.
        if (activeSlot >= 0 || !EraseSlots()) fault = true;
    }
    if (activeSlot < 0)
    {
        record.Magic = Magic;
        record.Version = 1;
    }
    retryStarted = now;
    retryDuration = FailureDelay(record.Failures);
    PrepareIdentity();
}

bool PinLock::EraseSlots()
{
    Record blank;
    Record check;
    memset(&blank, 0xFF, sizeof(blank));
    bool ok = true;
    for (uint8_t i = 0; i < 2 && ok; ++i)
        ok = writeRecord(i, reinterpret_cast<const uint8_t *>(&blank), sizeof(blank)) &&
             readRecord(i, reinterpret_cast<uint8_t *>(&check), sizeof(check)) &&
             Blank(&check, sizeof(check));
    clean(check);
    return ok;
}

bool PinLock::Provisioned() const
{
    return !fault && activeSlot >= 0;
}

bool PinLock::Save(Record &next)
{
    if (fault || record.Generation >= UINT32_MAX)
    {
        fault = true;
        authorized = false;
        return false;
    }
    next.Generation = record.Generation + 1;
    Digest(next);
    const uint8_t slot = activeSlot == 0 ? 1 : 0;
    Record check;
    bool ok = writeRecord(slot, reinterpret_cast<const uint8_t *>(&next), sizeof(next)) &&
              readRecord(slot, reinterpret_cast<uint8_t *>(&check), sizeof(check)) &&
              secure_compare(&next, &check, sizeof(next));
    clean(check);
    if (!ok)
    {
        fault = true;
        authorized = false;
        return false;
    }
    record = next;
    activeSlot = slot;
    return true;
}

PinLock::Result PinLock::Provision(const uint8_t seed[32])
{
    if (fault) return Result::StorageError;
    if (Provisioned()) return Result::Locked;
    uint8_t combined = 0;
    uint8_t different = 0;
    for (size_t i = 0; i < 32; ++i)
    {
        combined |= seed[i];
        different |= seed[i] ^ seed[0];
    }
    if (!combined || !different) return Result::Invalid;
    Record next = record;
    memcpy(next.Seed, seed, 32);
    bool ok = Save(next);
    clean(next);
    if (ok) PrepareIdentity();
    return ok ? Result::OK : Result::StorageError;
}

void PinLock::Derive(const char *label, uint64_t generation, uint8_t output[32]) const
{
    SHA256 hash;
    hash.resetHMAC(record.Seed, sizeof(record.Seed));
    hash.update(label, strlen(label));
    uint8_t counter[8];
    for (uint8_t i = 0; i < 8; ++i) counter[i] = generation >> (8 * i);
    hash.update(counter, sizeof(counter));
    hash.finalizeHMAC(record.Seed, sizeof(record.Seed), output, 32);
}

void PinLock::PrepareIdentity()
{
    clean(identity);
    if (!Provisioned()) return;
    uint8_t secret[32];
    Derive("SynapsePDM identity v1", 0, secret);
    Clamp(secret);
    Curve25519::eval(identity, secret, nullptr);
    clean(secret);
}

void PinLock::PublicKey(uint8_t output[32]) const
{
    memcpy(output, identity, sizeof(identity));
}

PinLock::Result PinLock::OpenSession(const uint8_t clientPublic[32], uint8_t devicePublic[32],
                                    uint8_t receiveKey[32], uint8_t transmitKey[32], uint8_t sessionID[8])
{
    if (fault) return Result::StorageError;
    if (!Provisioned()) return Result::NotProvisioned;
    if (pending) return Result::Busy;
    Lock();
    uint8_t secret[32], shared[32], zero[32] = {};
    Derive("SynapsePDM identity v1", 0, secret);
    Clamp(secret);
    bool valid = Curve25519::eval(shared, secret, clientPublic) &&
                 !secure_compare(shared, zero, sizeof(zero));
    clean(secret);
    if (!valid)
    {
        clean(shared);
        return Result::Invalid;
    }
    if (sessionEpoch == 0)
    {
        Record next = record;
        bool saved = Save(next);
        clean(next);
        if (!saved)
        {
            clean(shared);
            return Result::StorageError;
        }
        sessionEpoch = static_cast<uint32_t>(record.Generation);
    }
    if (sessionNumber == UINT32_MAX)
    {
        clean(shared);
        fault = true;
        return Result::StorageError;
    }
    ++sessionNumber;
    PublicKey(devicePublic);
    // One durable epoch per boot, not per unauthenticated handshake (EEPROM wear).
    for (uint8_t i = 0; i < 4; ++i)
    {
        sessionID[i] = sessionEpoch >> (8 * i);
        sessionID[i + 4] = sessionNumber >> (8 * i);
    }
    const char *labels[] = {"SynapsePDM client v1", "SynapsePDM device v1"};
    uint8_t *keys[] = {receiveKey, transmitKey};
    for (uint8_t i = 0; i < 2; ++i)
    {
        uint8_t info[84];
        const size_t labelLength = strlen(labels[i]);
        memcpy(info, labels[i], labelLength);
        memcpy(info + labelLength, clientPublic, 32);
        memcpy(info + labelLength + 32, devicePublic, 32);
        hkdf<SHA256>(keys[i], 32, shared, sizeof(shared), sessionID, 8, info, labelLength + 64);
    }
    clean(shared);
    return Result::OK;
}

uint32_t PinLock::FailureDelay(uint8_t failures)
{
    if (failures == 0) return 0;
    if (failures <= 3) return 1000;
    if (failures <= 6) return 30000;
    if (failures <= 9) return 300000;
    return 3600000;
}

uint32_t PinLock::RetryAfter(uint32_t now) const
{
    uint32_t elapsed = now - retryStarted;
    return elapsed >= retryDuration ? 0 : retryDuration - elapsed;
}

void PinLock::Lock()
{
    authorized = false;
    pending = false;
    clean(pin);
    clean(salt);
    clean(u);
    clean(derived);
    pinInner.clear();
    pinOuter.clear();
    rounds = 0;
}

bool PinLock::ValidPIN(const uint8_t *value, size_t length)
{
    if (length != 4) return false;
    for (size_t i = 0; i < length; ++i)
        if (value[i] < '0' || value[i] > '9') return false;
    return true;
}

PinLock::Result PinLock::ReserveAttempt(uint32_t now)
{
    if (RetryAfter(now)) return Result::Throttled;
    Record next = record;
    if (next.Failures < 10) ++next.Failures;
    bool ok = Save(next);
    clean(next);
    if (!ok) return Result::StorageError;
    // Reserve a failed attempt before checking a secret, so reset cannot undo a guess.
    retryStarted = now;
    retryDuration = FailureDelay(record.Failures);
    return Result::OK;
}

void PinLock::StartKdf(Operation operation, const uint8_t *newPIN)
{
    pendingOperation = operation;
    pending = true;
    rounds = 1;
    memcpy(pin, newPIN, sizeof(pin));
    if (operation == Operation::Unlock) memcpy(salt, record.Salt, sizeof(salt));
    else
    {
        uint8_t material[32];
        Derive("SynapsePDM PIN salt v1", record.Generation + 1, material);
        memcpy(salt, material, sizeof(salt));
        clean(material);
    }
    const uint8_t block[] = {0, 0, 0, 1};
    SHA256 hash;
    hash.resetHMAC(pin, sizeof(pin));
    hash.update(salt, sizeof(salt));
    hash.update(block, sizeof(block));
    hash.finalizeHMAC(pin, sizeof(pin), u, sizeof(u));
    memcpy(derived, u, sizeof(derived));

    // Cache the keyed SHA-256 prefixes: PBKDF2 uses the same HMAC key every round.
    uint8_t pad[SHA256::BLOCK_SIZE];
    memset(pad, 0x36, sizeof(pad));
    for (size_t i = 0; i < sizeof(pin); ++i) pad[i] ^= pin[i];
    pinInner.reset();
    pinInner.update(pad, sizeof(pad));
    memset(pad, 0x5C, sizeof(pad));
    for (size_t i = 0; i < sizeof(pin); ++i) pad[i] ^= pin[i];
    pinOuter.reset();
    pinOuter.update(pad, sizeof(pad));
    clean(pad);
}

void PinLock::MakeRecovery(Record &next, char output[RecoveryLength]) const
{
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";
    uint8_t material[32];
    Derive("SynapsePDM recovery key v1", record.Generation + 1, material);
    for (size_t i = 0; i < RecoveryLength; ++i) output[i] = alphabet[material[i] & 31];
    RecoveryDigest(next.Salt, reinterpret_cast<const uint8_t *>(output), next.RecoveryHash);
    clean(material);
}

PinLock::Result PinLock::Execute(Operation operation, const uint8_t *data, size_t length,
                                uint32_t now, char recovery[RecoveryLength])
{
    clean(recovery, RecoveryLength);
    if (fault) return Result::StorageError;
    if (!Provisioned()) return Result::NotProvisioned;
    if (pending) return Result::Busy;
    if (operation == Operation::Status && length == 0) return Result::OK;
    if (operation == Operation::Lock && length == 0)
    {
        Lock();
        return Result::OK;
    }
    if (operation == Operation::Unlock)
    {
        if (!ValidPIN(data, length) || !Enabled()) return Result::Invalid;
        Result result = ReserveAttempt(now);
        if (result != Result::OK) return result;
        StartKdf(operation, data);
        return Result::Pending;
    }
    if (operation == Operation::Recover)
    {
        if (length != RecoveryLength + 4 || !ValidPIN(data + RecoveryLength, 4) || !Enabled())
            return Result::Invalid;
        Result result = ReserveAttempt(now);
        if (result != Result::OK) return result;
        uint8_t hash[32];
        RecoveryDigest(record.Salt, data, hash);
        bool valid = secure_compare(hash, record.RecoveryHash, sizeof(hash));
        clean(hash);
        if (!valid) return Result::Locked;
        StartKdf(operation, data + RecoveryLength);
        return Result::Pending;
    }
    if (!Authorized()) return Result::Locked;
    if (operation == Operation::SetPIN)
    {
        if (!ValidPIN(data, length)) return Result::Invalid;
        StartKdf(operation, data);
        return Result::Pending;
    }
    if (length != 0) return Result::Invalid;
    Record next = record;
    if (operation == Operation::RotateRecovery && Enabled())
        MakeRecovery(next, recovery);
    else if (operation == Operation::Disable)
    {
        next.Enabled = 0;
        next.Failures = 0;
        clean(next.Salt);
        clean(next.PINHash);
        clean(next.RecoveryHash);
    }
    else
    {
        clean(next);
        return Result::Invalid;
    }
    bool ok = Save(next);
    clean(next);
    if (!ok) clean(recovery, RecoveryLength);
    if (ok && operation == Operation::Disable)
    {
        retryDuration = 0;
        authorized = false;
    }
    return ok ? Result::OK : Result::StorageError;
}

bool PinLock::Tick(uint32_t now, Result &result, char recovery[RecoveryLength])
{
    if (!pending) return false;
    clean(recovery, RecoveryLength);
    // Yield after 5 ms (plus the final round), retaining a hard cap if the clock stalls.
    const uint32_t started = micros();
    for (uint16_t batch = 0; batch < 256 && rounds < KdfIterations; ++batch)
    {
        uint8_t next[32];
        SHA256 inner = pinInner;
        inner.update(u, sizeof(u));
        inner.finalize(next, sizeof(next));
        SHA256 outer = pinOuter;
        outer.update(next, sizeof(next));
        outer.finalize(u, sizeof(u));
        for (size_t i = 0; i < sizeof(derived); ++i) derived[i] ^= u[i];
        clean(next);
        ++rounds;
        if (uint32_t(micros() - started) >= 5000) break;
    }
    if (rounds < KdfIterations) return false;
    if (pendingOperation == Operation::Unlock &&
        !secure_compare(derived, record.PINHash, sizeof(derived)))
    {
        result = Result::Locked;
        retryStarted = now;
        Lock();
        return true;
    }
    Record next = record;
    next.Failures = 0;
    if (pendingOperation != Operation::Unlock)
    {
        next.Enabled = 1;
        memcpy(next.Salt, salt, sizeof(salt));
        memcpy(next.PINHash, derived, sizeof(derived));
        MakeRecovery(next, recovery);
    }
    bool saved = Save(next);
    clean(next);
    Lock();
    if (saved)
    {
        authorized = true;
        retryDuration = 0;
        result = Result::OK;
    }
    else
    {
        clean(recovery, RecoveryLength);
        result = Result::StorageError;
    }
    return true;
}
