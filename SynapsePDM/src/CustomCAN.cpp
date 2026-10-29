#include "CustomCAN.h"
#include <math.h>
#include <string.h>

CustomCANConfiguration CustomCANConfig = {};
bool CustomCANCRCValid = false;
bool CustomCANConfigDirty = false;

struct CustomCANMessageRuntime
{
    uint32_t LastReceivedMs;
    CustomCANStatus Status;
};

struct CustomCANSignalRuntime
{
    float Value;
    CustomCANStatus Status;
};

struct CustomCANTxRuntime
{
    uint32_t NextDueMs;
    bool LastCondition;
    bool Pending;
};

static CustomCANMessageRuntime messageStates[MAX_CUSTOM_CAN_MESSAGES];
static CustomCANSignalRuntime signalStates[MAX_CUSTOM_CAN_SIGNALS];
static CustomCANTxRuntime txStates[MAX_CUSTOM_CAN_MESSAGES];
static_assert(sizeof(messageStates) + sizeof(signalStates) + sizeof(txStates) == 2400,
              "Custom CAN runtime RAM budget");

static CustomCANMessageConfig DefaultMessage()
{
    return {0, 1000, 8, 0, 0};
}

static CustomCANSignalConfig DefaultSignal()
{
    return {0, 0, 1, 0, 1.0f, 0.0f, 0};
}

static bool ValidChannelSource(uint8_t source)
{
    return source < CUSTOM_CAN_TX_CHANNEL_SOURCES || source == CUSTOM_CAN_SOURCE_CONSTANT;
}

static bool ValidMessage(const CustomCANMessageConfig &config)
{
    const uint8_t validFlags = CUSTOM_CAN_MESSAGE_ENABLED | CUSTOM_CAN_MESSAGE_EXTENDED |
                               CUSTOM_CAN_MESSAGE_TX | CUSTOM_CAN_MESSAGE_TX_PERIODIC;
    bool tx = config.Flags & CUSTOM_CAN_MESSAGE_TX;
    if ((config.Flags & ~validFlags) ||
        ((config.Flags & CUSTOM_CAN_MESSAGE_TX_PERIODIC) && !tx) ||
        config.ID > ((config.Flags & CUSTOM_CAN_MESSAGE_EXTENDED) ? 0x1FFFFFFFUL : 0x7FFUL) ||
        config.TimeoutMs > MAX_CUSTOM_CAN_TIMEOUT_MS || config.DLC > 8 ||
        !ValidChannelSource(config.TxTrigger))
    {
        return false;
    }
    if (tx)
    {
        // TimeoutMs is the transmit period; data-less frames (DLC 0) are legal for TX only.
        return !(config.Flags & CUSTOM_CAN_MESSAGE_TX_PERIODIC) || config.TimeoutMs > 0;
    }
    // TimeoutMs 0 = never expire for RX messages.
    return config.DLC > 0;
}

static bool ValidSignal(const CustomCANSignalConfig &config, uint8_t dlc)
{
    if (config.MessageIndex >= MAX_CUSTOM_CAN_MESSAGES ||
        (config.Flags & ~(CUSTOM_CAN_SIGNAL_ENABLED | CUSTOM_CAN_SIGNAL_MOTOROLA | CUSTOM_CAN_SIGNAL_SIGNED)) ||
        config.StartBit >= 64 || config.BitLength == 0 || config.BitLength > 64 ||
        !isfinite(config.Factor) || !isfinite(config.Offset) ||
        !ValidChannelSource(config.ValueSource))
    {
        return false;
    }
    int bit = config.StartBit;
    for (uint8_t i = 0; i < config.BitLength; i++)
    {
        if (bit < 0 || bit >= dlc * 8)
        {
            return false;
        }
        // DBC Motorola start bits identify the MSB, with sawtooth byte numbering.
        bit = (config.Flags & CUSTOM_CAN_SIGNAL_MOTOROLA) ? ((bit % 8 == 0) ? bit + 15 : bit - 1) : bit + 1;
    }
    return true;
}

// A signal on a TX message needs a usable encode: raw = (value - Offset) / Factor,
// except constant sources, which encode round(Offset) directly.
static bool ValidSignalForMessage(const CustomCANSignalConfig &config, const CustomCANMessageConfig &message)
{
    if (!ValidSignal(config, message.DLC))
    {
        return false;
    }
    return !(message.Flags & CUSTOM_CAN_MESSAGE_TX) || !(config.Flags & CUSTOM_CAN_SIGNAL_ENABLED) ||
           config.ValueSource == CUSTOM_CAN_SOURCE_CONSTANT || config.Factor != 0.0f;
}

void ResetCustomCANRuntime()
{
    for (auto &state : messageStates) state = {0, CUSTOM_CAN_NOT_RECEIVED};
    for (auto &state : signalStates) state = {0.0f, CUSTOM_CAN_NOT_RECEIVED};
    for (auto &state : txStates) state = {0, false, false};
}

void InitialiseCustomCANData()
{
    CustomCANCRCValid = false;
    CustomCANConfig = {};
    for (auto &message : CustomCANConfig.Messages)
    {
        message = DefaultMessage();
    }
    for (auto &signal : CustomCANConfig.Signals)
    {
        signal = DefaultSignal();
    }
    CustomCANConfigDirty = false;
    ResetCustomCANRuntime();
}

bool ValidateCustomCANConfig()
{
    for (uint8_t i = 0; i < MAX_CUSTOM_CAN_MESSAGES; i++)
    {
        const auto &message = CustomCANConfig.Messages[i];
        if (!ValidMessage(message))
        {
            return false;
        }
        if (!(message.Flags & CUSTOM_CAN_MESSAGE_ENABLED) || (message.Flags & CUSTOM_CAN_MESSAGE_TX))
        {
            // TX definitions may legitimately share an ID, e.g. on/off command frames.
            continue;
        }
        for (uint8_t j = 0; j < i; j++)
        {
            const auto &other = CustomCANConfig.Messages[j];
            if ((other.Flags & CUSTOM_CAN_MESSAGE_ENABLED) && !(other.Flags & CUSTOM_CAN_MESSAGE_TX) &&
                message.ID == other.ID &&
                !((message.Flags ^ other.Flags) & CUSTOM_CAN_MESSAGE_EXTENDED))
            {
                return false;
            }
        }
    }
    for (const auto &signal : CustomCANConfig.Signals)
    {
        if (signal.MessageIndex >= MAX_CUSTOM_CAN_MESSAGES ||
            !ValidSignalForMessage(signal, CustomCANConfig.Messages[signal.MessageIndex]))
        {
            return false;
        }
    }
    return true;
}

bool SetCustomCANMessage(uint8_t index, const CustomCANMessageConfig &config)
{
    if (index >= MAX_CUSTOM_CAN_MESSAGES || !ValidMessage(config))
    {
        return false;
    }
    const auto previous = CustomCANConfig.Messages[index];
    CustomCANConfig.Messages[index] = config;
    if (!ValidateCustomCANConfig())
    {
        CustomCANConfig.Messages[index] = previous;
        return false;
    }
    if (memcmp(&previous, &config, sizeof(config)) != 0)
    {
        messageStates[index] = {0, CUSTOM_CAN_NOT_RECEIVED};
        txStates[index] = {0, false, false};
        for (uint8_t i = 0; i < MAX_CUSTOM_CAN_SIGNALS; i++)
            if (CustomCANConfig.Signals[i].MessageIndex == index)
            {
                signalStates[i] = {0.0f, CUSTOM_CAN_NOT_RECEIVED};
            }
        CustomCANConfigDirty = true;
    }
    return true;
}

bool SetCustomCANSignal(uint8_t index, const CustomCANSignalConfig &config)
{
    if (index >= MAX_CUSTOM_CAN_SIGNALS || config.MessageIndex >= MAX_CUSTOM_CAN_MESSAGES ||
        !ValidSignalForMessage(config, CustomCANConfig.Messages[config.MessageIndex]))
    {
        return false;
    }
    if (memcmp(&CustomCANConfig.Signals[index], &config, sizeof(config)) != 0)
    {
        CustomCANConfig.Signals[index] = config;
        signalStates[index] = {0.0f, CUSTOM_CAN_NOT_RECEIVED};
        CustomCANConfigDirty = true;
    }
    return true;
}

uint8_t GetCustomCANWriteSize(uint8_t type, uint8_t parameter)
{
    if (parameter == CUSTOM_CAN_PARAMETER_CLEAR)
    {
        return 0;
    }
    if (parameter == CUSTOM_CAN_PARAMETER_RECORD)
    {
        return type == 0 ? sizeof(CustomCANMessageConfig) : sizeof(CustomCANSignalConfig);
    }
    return sizeof(uint32_t);
}

bool ApplyCustomCANWrite(uint8_t type, uint8_t index, uint8_t parameter, const uint8_t *data, uint16_t length)
{
    if (type > 1 || length != GetCustomCANWriteSize(type, parameter) || (length > 0 && data == nullptr))
    {
        return false;
    }
    if (parameter == CUSTOM_CAN_PARAMETER_CLEAR || parameter == CUSTOM_CAN_PARAMETER_RECORD)
    {
        if (type == 0)
        {
            auto config = DefaultMessage();
            if (length > 0) memcpy(&config, data, sizeof(config));
            return SetCustomCANMessage(index, config);
        }
        auto config = DefaultSignal();
        if (length > 0) memcpy(&config, data, sizeof(config));
        return SetCustomCANSignal(index, config);
    }
    uint32_t value;
    memcpy(&value, data, sizeof(value));
    return SetCustomCANParameter(type, index, parameter, value);
}

static bool DecodeSignal(const CustomCANSignalConfig &config, const uint8_t *data, uint8_t length, float *value)
{
    if (!ValidSignal(config, length))
    {
        return false;
    }
    uint64_t raw = 0;
    int bit = config.StartBit;
    for (uint8_t i = 0; i < config.BitLength; i++)
    {
        uint64_t next = (data[bit / 8] >> (bit % 8)) & 1U;
        if (config.Flags & CUSTOM_CAN_SIGNAL_MOTOROLA)
        {
            raw = (raw << 1) | next;
            bit = (bit % 8 == 0) ? bit + 15 : bit - 1;
        }
        else
        {
            raw |= next << i;
            bit++;
        }
    }
    double decoded = static_cast<double>(raw);
    if ((config.Flags & CUSTOM_CAN_SIGNAL_SIGNED) && (raw & (UINT64_C(1) << (config.BitLength - 1))))
    {
        // Convert the two's-complement magnitude without shifting by 64 or
        // subtracting nearly equal floating-point values for small negatives.
        uint64_t mask = UINT64_MAX >> (64 - config.BitLength);
        decoded = -static_cast<double>(((~raw) & mask) + 1);
    }
    *value = static_cast<float>(decoded * config.Factor + config.Offset);
    return isfinite(*value);
}

void ProcessCustomCANFrame(uint32_t id, bool extended, bool remote, const uint8_t *data, uint8_t length, uint32_t now)
{
    for (uint8_t messageIndex = 0; messageIndex < MAX_CUSTOM_CAN_MESSAGES; messageIndex++)
    {
        const auto &message = CustomCANConfig.Messages[messageIndex];
        if (!(message.Flags & CUSTOM_CAN_MESSAGE_ENABLED) || (message.Flags & CUSTOM_CAN_MESSAGE_TX) ||
            message.ID != id ||
            !!(message.Flags & CUSTOM_CAN_MESSAGE_EXTENDED) != extended)
        {
            continue;
        }
        bool validFrame = !remote && data != nullptr && length == message.DLC && length <= 8;
        messageStates[messageIndex].Status = validFrame ? CUSTOM_CAN_VALID : CUSTOM_CAN_INVALID_FRAME;
        if (validFrame)
        {
            messageStates[messageIndex].LastReceivedMs = now;
        }
        for (uint8_t i = 0; i < MAX_CUSTOM_CAN_SIGNALS; i++)
        {
            const auto &signal = CustomCANConfig.Signals[i];
            if (!(signal.Flags & CUSTOM_CAN_SIGNAL_ENABLED) || signal.MessageIndex != messageIndex)
            {
                continue;
            }
            auto &state = signalStates[i];
            state.Value = 0.0f;
            state.Status = !validFrame ? CUSTOM_CAN_INVALID_FRAME :
                (DecodeSignal(signal, data, length, &state.Value) ? CUSTOM_CAN_VALID : CUSTOM_CAN_DECODE_ERROR);
        }
    }
}

CustomCANStatus GetCustomCANMessageStatus(uint8_t index, uint32_t now)
{
    if (index >= MAX_CUSTOM_CAN_MESSAGES)
    {
        return CUSTOM_CAN_INVALID_CONFIG;
    }
    const auto &message = CustomCANConfig.Messages[index];
    if (!(message.Flags & CUSTOM_CAN_MESSAGE_ENABLED))
    {
        return CUSTOM_CAN_DISABLED;
    }
    if (!ValidMessage(message))
    {
        return CUSTOM_CAN_INVALID_CONFIG;
    }
    if (message.Flags & CUSTOM_CAN_MESSAGE_TX)
    {
        // TX definitions are never received; their signals are not readable.
        return CUSTOM_CAN_DISABLED;
    }
    auto &state = messageStates[index];
    if (state.Status != CUSTOM_CAN_VALID)
    {
        return state.Status;
    }
    if (message.TimeoutMs > 0 && static_cast<uint32_t>(now - state.LastReceivedMs) >= message.TimeoutMs)
    {
        state.Status = CUSTOM_CAN_TIMEOUT;
    }
    return state.Status;
}

void UpdateCustomCANTimeouts(uint32_t now)
{
    for (uint8_t i = 0; i < MAX_CUSTOM_CAN_MESSAGES; i++)
    {
        if (messageStates[i].Status == CUSTOM_CAN_VALID)
        {
            GetCustomCANMessageStatus(i, now);
        }
    }
}

// Inverse of DecodeSignal: scale the physical value, clamp to the representable
// raw range, and pack Intel or Motorola (DBC sawtooth) bits into the frame.
static bool EncodeSignal(const CustomCANSignalConfig &config, float value, uint8_t *data, uint8_t dlc)
{
    if (!ValidSignal(config, dlc))
    {
        return false;
    }
    double scaled;
    if (config.ValueSource == CUSTOM_CAN_SOURCE_CONSTANT)
    {
        scaled = config.Offset;
    }
    else
    {
        if (config.Factor == 0.0f || !isfinite(value))
        {
            return false;
        }
        scaled = (static_cast<double>(value) - config.Offset) / config.Factor;
        if (!isfinite(scaled))
        {
            return false;
        }
    }
    bool isSigned = config.Flags & CUSTOM_CAN_SIGNAL_SIGNED;
    uint64_t mask = (config.BitLength >= 64) ? UINT64_MAX : ((UINT64_C(1) << config.BitLength) - 1);
    double rounded = nearbyint(scaled);
    uint64_t raw;
    if (isSigned)
    {
        // Compare against the limits before converting so 64-bit extremes stay defined behaviour.
        double maxPositive = ldexp(1.0, config.BitLength - 1) - 1.0;
        double minNegative = -ldexp(1.0, config.BitLength - 1);
        if (rounded >= maxPositive) raw = mask >> 1;
        else if (rounded <= minNegative) raw = (mask >> 1) + 1;
        else raw = static_cast<uint64_t>(static_cast<int64_t>(rounded)) & mask;
    }
    else
    {
        double maxValue = ldexp(1.0, config.BitLength) - 1.0;
        if (rounded >= maxValue) raw = mask;
        else if (rounded <= 0.0) raw = 0;
        else raw = static_cast<uint64_t>(rounded) & mask;
    }
    int bit = config.StartBit;
    for (uint8_t i = 0; i < config.BitLength; i++)
    {
        bool motorola = config.Flags & CUSTOM_CAN_SIGNAL_MOTOROLA;
        uint64_t bitValue = motorola ? (raw >> (config.BitLength - 1 - i)) & 1U : (raw >> i) & 1U;
        if (bitValue)
        {
            data[bit / 8] |= static_cast<uint8_t>(1U << (bit % 8));
        }
        else
        {
            data[bit / 8] &= static_cast<uint8_t>(~(1U << (bit % 8)));
        }
        bit = motorola ? ((bit % 8 == 0) ? bit + 15 : bit - 1) : bit + 1;
    }
    return true;
}

void ServiceCustomCANTransmits(uint32_t now, CustomCANValueReader readValue, CustomCANFrameSender send)
{
    if (readValue == nullptr || send == nullptr)
    {
        return;
    }
    for (uint8_t i = 0; i < MAX_CUSTOM_CAN_MESSAGES; i++)
    {
        const auto &message = CustomCANConfig.Messages[i];
        auto &state = txStates[i];
        const uint8_t txEnabled = CUSTOM_CAN_MESSAGE_ENABLED | CUSTOM_CAN_MESSAGE_TX;
        if ((message.Flags & txEnabled) != txEnabled || !ValidMessage(message))
        {
            state = {0, false, false};
            continue;
        }
        bool condition = true;
        if (message.TxTrigger != CUSTOM_CAN_SOURCE_CONSTANT)
        {
            float trigger = 0.0f;
            condition = readValue(message.TxTrigger, &trigger) && trigger != 0.0f;
        }
        bool rising = condition && !state.LastCondition;
        state.LastCondition = condition;
        if (message.Flags & CUSTOM_CAN_MESSAGE_TX_PERIODIC)
        {
            if (!condition)
            {
                state.Pending = false;
                continue;
            }
            if (rising || static_cast<int32_t>(now - state.NextDueMs) >= 0)
            {
                state.Pending = true;
            }
        }
        else if (rising)
        {
            state.Pending = true;
        }
        if (!state.Pending)
        {
            continue;
        }
        uint8_t data[8] = {0};
        bool encoded = true;
        for (uint8_t j = 0; encoded && j < MAX_CUSTOM_CAN_SIGNALS; j++)
        {
            const auto &signal = CustomCANConfig.Signals[j];
            if (!(signal.Flags & CUSTOM_CAN_SIGNAL_ENABLED) || signal.MessageIndex != i)
            {
                continue;
            }
            float value = 0.0f;
            if (signal.ValueSource != CUSTOM_CAN_SOURCE_CONSTANT && !readValue(signal.ValueSource, &value))
            {
                encoded = false;
                break;
            }
            encoded = EncodeSignal(signal, value, data, message.DLC);
        }
        // Unavailable sources or a full transmit queue leave the frame pending for the next service call.
        if (encoded && send(message.ID, message.Flags & CUSTOM_CAN_MESSAGE_EXTENDED, data, message.DLC))
        {
            state.Pending = false;
            if (message.Flags & CUSTOM_CAN_MESSAGE_TX_PERIODIC)
            {
                state.NextDueMs = now + message.TimeoutMs;
            }
        }
    }
}

CustomCANStatus GetCustomCANSignalStatus(uint8_t index, uint32_t now)
{
    if (index >= MAX_CUSTOM_CAN_SIGNALS)
    {
        return CUSTOM_CAN_INVALID_CONFIG;
    }
    const auto &signal = CustomCANConfig.Signals[index];
    if (!(signal.Flags & CUSTOM_CAN_SIGNAL_ENABLED))
    {
        return CUSTOM_CAN_DISABLED;
    }
    if (signal.MessageIndex >= MAX_CUSTOM_CAN_MESSAGES ||
        !ValidSignalForMessage(signal, CustomCANConfig.Messages[signal.MessageIndex]))
    {
        return CUSTOM_CAN_INVALID_CONFIG;
    }
    CustomCANStatus messageStatus = GetCustomCANMessageStatus(signal.MessageIndex, now);
    return messageStatus == CUSTOM_CAN_VALID ? signalStates[index].Status : messageStatus;
}

bool ReadCustomCANSignal(uint8_t index, uint32_t now, float *value)
{
    if (value == nullptr || GetCustomCANSignalStatus(index, now) != CUSTOM_CAN_VALID)
    {
        return false;
    }
    *value = signalStates[index].Value;
    return true;
}

bool SetCustomCANParameter(uint8_t type, uint8_t index, uint8_t parameter, uint32_t value)
{
    if (type == 0 && index < MAX_CUSTOM_CAN_MESSAGES)
    {
        auto config = CustomCANConfig.Messages[index];
        switch (parameter)
        {
        case 0: config.ID = value; break;
        case 1:
            if (value > MAX_CUSTOM_CAN_TIMEOUT_MS)
            {
                return false;
            }
            config.TimeoutMs = static_cast<uint16_t>(value);
            break;
        case 2:
            if (value > 8)
            {
                return false;
            }
            config.DLC = static_cast<uint8_t>(value);
            break;
        case 3:
            if (value > UINT8_MAX)
            {
                return false;
            }
            config.Flags = static_cast<uint8_t>(value);
            break;
        case 5:
            if (value > UINT8_MAX)
            {
                return false;
            }
            config.TxTrigger = static_cast<uint8_t>(value);
            break;
        default: return false;
        }
        return SetCustomCANMessage(index, config);
    }
    if (type == 1 && index < MAX_CUSTOM_CAN_SIGNALS)
    {
        auto config = CustomCANConfig.Signals[index];
        if ((parameter <= 3 || parameter == 8) && value > UINT8_MAX)
        {
            return false;
        }
        switch (parameter)
        {
        case 0: config.MessageIndex = static_cast<uint8_t>(value); break;
        case 1: config.StartBit = static_cast<uint8_t>(value); break;
        case 2: config.BitLength = static_cast<uint8_t>(value); break;
        case 3: config.Flags = static_cast<uint8_t>(value); break;
        case 4: memcpy(&config.Factor, &value, sizeof(value)); break;
        case 5: memcpy(&config.Offset, &value, sizeof(value)); break;
        case 8: config.ValueSource = static_cast<uint8_t>(value); break;
        default: return false;
        }
        return SetCustomCANSignal(index, config);
    }
    return false;
}

bool GetCustomCANParameter(uint8_t type, uint8_t index, uint8_t parameter, uint32_t now, uint32_t *value)
{
    if (value == nullptr)
    {
        return false;
    }
    *value = 0;
    if (type == 0 && index < MAX_CUSTOM_CAN_MESSAGES)
    {
        const auto &config = CustomCANConfig.Messages[index];
        switch (parameter)
        {
        case 0: *value = config.ID; break;
        case 1: *value = config.TimeoutMs; break;
        case 2: *value = config.DLC; break;
        case 3: *value = config.Flags; break;
        case 4: *value = GetCustomCANMessageStatus(index, now); break;
        case 5: *value = config.TxTrigger; break;
        default: return false;
        }
        return true;
    }
    if (type == 1 && index < MAX_CUSTOM_CAN_SIGNALS)
    {
        const auto &config = CustomCANConfig.Signals[index];
        switch (parameter)
        {
        case 0: *value = config.MessageIndex; break;
        case 1: *value = config.StartBit; break;
        case 2: *value = config.BitLength; break;
        case 3: *value = config.Flags; break;
        case 4: memcpy(value, &config.Factor, sizeof(*value)); break;
        case 5: memcpy(value, &config.Offset, sizeof(*value)); break;
        case 6:
        {
            float result = 0.0f;
            if (!ReadCustomCANSignal(index, now, &result))
            {
                return false;
            }
            memcpy(value, &result, sizeof(*value));
            break;
        }
        case 7: *value = GetCustomCANSignalStatus(index, now); break;
        case 8: *value = config.ValueSource; break;
        default: return false;
        }
        return true;
    }
    return false;
}
