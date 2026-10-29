#ifndef CustomCAN_H
#define CustomCAN_H

#include <stdint.h>

#define MAX_CUSTOM_CAN_SIGNALS 100
#define MAX_CUSTOM_CAN_MESSAGES 100
#define MAX_CUSTOM_CAN_TIMEOUT_MS 60000
// Mirrors NUM_CHANNELS; compile-time checked where both headers are visible.
#define CUSTOM_CAN_TX_CHANNEL_SOURCES 14
#define CUSTOM_CAN_SOURCE_CONSTANT 255
#define CUSTOM_CAN_PARAMETER_CLEAR 254
#define CUSTOM_CAN_PARAMETER_RECORD 255

enum CustomCANMessageFlags : uint8_t
{
    CUSTOM_CAN_MESSAGE_ENABLED = 1,
    CUSTOM_CAN_MESSAGE_EXTENDED = 2,
    CUSTOM_CAN_MESSAGE_TX = 4,
    CUSTOM_CAN_MESSAGE_TX_PERIODIC = 8
};

enum CustomCANSignalFlags : uint8_t
{
    CUSTOM_CAN_SIGNAL_ENABLED = 1,
    CUSTOM_CAN_SIGNAL_MOTOROLA = 2,
    CUSTOM_CAN_SIGNAL_SIGNED = 4
};

struct __attribute__((packed)) CustomCANMessageConfig
{
    uint32_t ID;
    /// RX: timeout in ms, 0 = never expire. TX periodic: transmit period in ms.
    uint16_t TimeoutMs;
    uint8_t DLC;
    uint8_t Flags;
    /// TX only: channel whose output gates transmission, or CUSTOM_CAN_SOURCE_CONSTANT for always-true.
    uint8_t TxTrigger;
};

struct __attribute__((packed)) CustomCANSignalConfig
{
    uint8_t MessageIndex;
    uint8_t StartBit;
    uint8_t BitLength;
    uint8_t Flags;
    float Factor;
    float Offset;
    /// TX only: channel whose output is encoded, or CUSTOM_CAN_SOURCE_CONSTANT (Offset holds the raw value).
    uint8_t ValueSource;
};

struct CustomCANConfiguration
{
    CustomCANMessageConfig Messages[MAX_CUSTOM_CAN_MESSAGES];
    CustomCANSignalConfig Signals[MAX_CUSTOM_CAN_SIGNALS];
};

static_assert(sizeof(float) == 4, "Custom CAN requires 32-bit floats");
static_assert(sizeof(CustomCANMessageConfig) == 9, "Custom CAN message EEPROM layout");
static_assert(sizeof(CustomCANSignalConfig) == 13, "Custom CAN signal EEPROM layout");
static_assert(sizeof(CustomCANConfiguration) == 2200, "Custom CAN EEPROM payload layout");

enum CustomCANStatus : uint8_t
{
    CUSTOM_CAN_VALID = 0,
    CUSTOM_CAN_DISABLED,
    CUSTOM_CAN_NOT_RECEIVED,
    CUSTOM_CAN_TIMEOUT,
    CUSTOM_CAN_INVALID_FRAME,
    CUSTOM_CAN_DECODE_ERROR,
    CUSTOM_CAN_INVALID_CONFIG
};

extern CustomCANConfiguration CustomCANConfig;
extern bool CustomCANCRCValid;
extern bool CustomCANConfigDirty;

/// @brief Reads one channel output value for TX triggers/sources. Returns false when unavailable.
typedef bool (*CustomCANValueReader)(uint8_t channelIndex, float *value);
/// @brief Queues one CAN frame for transmission. Returns false when the frame could not be queued.
typedef bool (*CustomCANFrameSender)(uint32_t id, bool extended, const uint8_t *data, uint8_t dlc);

void InitialiseCustomCANData();
void ResetCustomCANRuntime();
void UpdateCustomCANTimeouts(uint32_t now);
void ServiceCustomCANTransmits(uint32_t now, CustomCANValueReader readValue, CustomCANFrameSender send);
bool ValidateCustomCANConfig();
bool SetCustomCANMessage(uint8_t index, const CustomCANMessageConfig &config);
bool SetCustomCANSignal(uint8_t index, const CustomCANSignalConfig &config);
uint8_t GetCustomCANWriteSize(uint8_t type, uint8_t parameter);
bool ApplyCustomCANWrite(uint8_t type, uint8_t index, uint8_t parameter, const uint8_t *data, uint16_t length);
void ProcessCustomCANFrame(uint32_t id, bool extended, bool remote, const uint8_t *data, uint8_t length, uint32_t now);
CustomCANStatus GetCustomCANMessageStatus(uint8_t index, uint32_t now);
CustomCANStatus GetCustomCANSignalStatus(uint8_t index, uint32_t now);
bool ReadCustomCANSignal(uint8_t index, uint32_t now, float *value);
bool SetCustomCANParameter(uint8_t type, uint8_t index, uint8_t parameter, uint32_t value);
bool GetCustomCANParameter(uint8_t type, uint8_t index, uint8_t parameter, uint32_t now, uint32_t *value);

#endif
