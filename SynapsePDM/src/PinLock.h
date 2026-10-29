#ifndef PIN_LOCK_H
#define PIN_LOCK_H

#include <stddef.h>
#include <stdint.h>
#include <SHA256.h>

class PinLock
{
public:
    static constexpr uint32_t KdfIterations = 10000;
    static constexpr size_t RecordSize = 160;
    static constexpr size_t RecoveryLength = 32;
    enum class Result : uint8_t
    {
        OK, Invalid, Locked, Throttled, StorageError, NotProvisioned, Busy, Pending
    };
    enum class Operation : uint8_t
    {
        Info = 0, Provision = 1, Hello = 2, Unlock = 3, SetPIN = 4,
        Recover = 5, RotateRecovery = 6, Disable = 7, Lock = 8, Status = 9
    };
    using ReadRecord = bool (*)(uint8_t slot, uint8_t *data, size_t length);
    using WriteRecord = bool (*)(uint8_t slot, const uint8_t *data, size_t length);

    PinLock(ReadRecord read, WriteRecord write);
    void Begin(uint32_t now);
    bool Enabled() const { return fault || record.Enabled != 0; }
    bool Faulted() const { return fault; }
    bool Provisioned() const;
    bool Authorized() const { return !fault && (!Enabled() || authorized); }
    bool Busy() const { return pending; }
    uint32_t RetryAfter(uint32_t now) const;
    void Lock();
    Result Provision(const uint8_t seed[32]);
    Result OpenSession(const uint8_t clientPublic[32], uint8_t devicePublic[32],
                       uint8_t receiveKey[32], uint8_t transmitKey[32], uint8_t sessionID[8]);
    Result Execute(Operation operation, const uint8_t *data, size_t length, uint32_t now,
                   char recovery[RecoveryLength]);
    bool Tick(uint32_t now, Result &result, char recovery[RecoveryLength]);
    void PublicKey(uint8_t output[32]) const;

private:
    struct Record
    {
        uint32_t Magic;
        uint8_t Version;
        uint8_t Enabled;
        uint8_t Failures;
        uint8_t Reserved;
        uint64_t Generation;
        uint8_t Seed[32];
        uint8_t Salt[16];
        uint8_t PINHash[32];
        uint8_t RecoveryHash[32];
        uint8_t Digest[32];
    };
    static_assert(sizeof(Record) == RecordSize, "PIN EEPROM layout");
    Record record = {};
    ReadRecord readRecord;
    WriteRecord writeRecord;
    int8_t activeSlot = -1;
    bool fault = true;
    bool authorized = false;
    bool pending = false;
    Operation pendingOperation = Operation::Info;
    uint32_t retryStarted = 0;
    uint32_t retryDuration = 0;
    uint32_t rounds = 0;
    uint8_t pin[4] = {};
    uint8_t salt[16] = {};
    uint8_t u[32] = {};
    uint8_t derived[32] = {};
    SHA256 pinInner;
    SHA256 pinOuter;
    uint8_t identity[32] = {};
    uint32_t sessionEpoch = 0;
    uint32_t sessionNumber = 0;

    bool Save(Record &next);
    bool EraseSlots();
    Result ReserveAttempt(uint32_t now);
    void StartKdf(Operation operation, const uint8_t *newPIN);
    void MakeRecovery(Record &next, char output[RecoveryLength]) const;
    void Derive(const char *label, uint64_t generation, uint8_t output[32]) const;
    void PrepareIdentity();
    static void Digest(Record &value);
    static bool Valid(const Record &value);
    static bool ValidPIN(const uint8_t *value, size_t length);
    static uint32_t FailureDelay(uint8_t failures);
};

#endif
