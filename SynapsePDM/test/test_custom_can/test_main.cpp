#include <unity.h>
#include <float.h>
#include <math.h>
#include "../../src/CustomCAN.cpp"

#ifdef ARDUINO
#include <Arduino.h>
#endif

static void ResetTxHarness();

void setUp()
{
    InitialiseCustomCANData();
    ResetTxHarness();
}

void tearDown() {}

static void Message(uint8_t index = 0, uint32_t id = 0x180, bool extended = false)
{
    CustomCANMessageConfig config = {id, 1000, 8,
        static_cast<uint8_t>(CUSTOM_CAN_MESSAGE_ENABLED | (extended ? CUSTOM_CAN_MESSAGE_EXTENDED : 0))};
    TEST_ASSERT_TRUE(SetCustomCANMessage(index, config));
}

static void Signal(uint8_t index, uint8_t message, uint8_t start, uint8_t length,
                   uint8_t flags = 0, float factor = 1.0f, float offset = 0.0f)
{
    CustomCANSignalConfig config = {message, start, length,
        static_cast<uint8_t>(CUSTOM_CAN_SIGNAL_ENABLED | flags), factor, offset};
    TEST_ASSERT_TRUE(SetCustomCANSignal(index, config));
}

static float Value(uint8_t index, uint32_t now = 100)
{
    float value = NAN;
    TEST_ASSERT_TRUE(ReadCustomCANSignal(index, now, &value));
    return value;
}

// --- TX harness: stub channel outputs and captured frames ---

static float txChannelValues[CUSTOM_CAN_TX_CHANNEL_SOURCES];
static bool txChannelValid[CUSTOM_CAN_TX_CHANNEL_SOURCES];
struct CapturedFrame
{
    uint32_t ID;
    bool Extended;
    uint8_t DLC;
    uint8_t Data[8];
};
static CapturedFrame txFrames[16];
static uint8_t txFrameCount;
static bool txSendResult;

static void ResetTxHarness()
{
    memset(txChannelValues, 0, sizeof(txChannelValues));
    memset(txChannelValid, 0, sizeof(txChannelValid));
    memset(txFrames, 0, sizeof(txFrames));
    txFrameCount = 0;
    txSendResult = true;
}

static bool StubReader(uint8_t channelIndex, float *value)
{
    if (channelIndex >= CUSTOM_CAN_TX_CHANNEL_SOURCES || !txChannelValid[channelIndex])
    {
        return false;
    }
    *value = txChannelValues[channelIndex];
    return true;
}

static bool StubSender(uint32_t id, bool extended, const uint8_t *data, uint8_t dlc)
{
    if (!txSendResult)
    {
        return false;
    }
    TEST_ASSERT_LESS_THAN_UINT8(16, txFrameCount);
    txFrames[txFrameCount].ID = id;
    txFrames[txFrameCount].Extended = extended;
    txFrames[txFrameCount].DLC = dlc;
    memcpy(txFrames[txFrameCount].Data, data, dlc);
    txFrameCount++;
    return true;
}

static void TxMessage(uint8_t index, uint32_t id, uint8_t dlc, uint8_t trigger,
                      bool periodic = false, uint16_t period = 1000)
{
    CustomCANMessageConfig config = {id, period, dlc,
        static_cast<uint8_t>(CUSTOM_CAN_MESSAGE_ENABLED | CUSTOM_CAN_MESSAGE_TX |
            (periodic ? CUSTOM_CAN_MESSAGE_TX_PERIODIC : 0)), trigger};
    TEST_ASSERT_TRUE(SetCustomCANMessage(index, config));
}

static void TxSignal(uint8_t index, uint8_t message, uint8_t start, uint8_t length, uint8_t source,
                     uint8_t flags = 0, float factor = 1.0f, float offset = 0.0f)
{
    CustomCANSignalConfig config = {message, start, length,
        static_cast<uint8_t>(CUSTOM_CAN_SIGNAL_ENABLED | flags), factor, offset, source};
    TEST_ASSERT_TRUE(SetCustomCANSignal(index, config));
}

void test_layout_defaults_and_capacity()
{
    TEST_ASSERT_EQUAL_UINT32(2200, sizeof(CustomCANConfiguration));
    TEST_ASSERT_TRUE(ValidateCustomCANConfig());
    TEST_ASSERT_EQUAL(CUSTOM_CAN_DISABLED, GetCustomCANSignalStatus(0, 0));
    TEST_ASSERT_FALSE(ReadCustomCANSignal(100, 0, nullptr));
    for (uint8_t i = 0; i < 100; i++)
    {
        Message(i, 0x180 + i);
        Signal(i, i, 0, 8);
        uint8_t data[8] = {i};
        ProcessCustomCANFrame(0x180 + i, false, false, data, 8, 100);
    }
    for (uint8_t i = 0; i < 100; i++) TEST_ASSERT_EQUAL_FLOAT(i, Value(i));
}

void test_shared_frame_unsigned_signed_and_scaling()
{
    Message();
    Signal(0, 0, 0, 16, 0, 0.25f, -10.0f);
    Signal(1, 0, 16, 8, CUSTOM_CAN_SIGNAL_SIGNED);
    Signal(99, 0, 24, 8);
    uint8_t data[8] = {0xA0, 0x0F, 0xF6, 0x7B};
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    TEST_ASSERT_EQUAL_FLOAT(990.0f, Value(0));
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, Value(1));
    TEST_ASSERT_EQUAL_FLOAT(123.0f, Value(99));
}

void test_little_endian_cross_byte_and_bit_boundaries()
{
    Message();
    Signal(0, 0, 5, 12);
    Signal(1, 0, 63, 1, CUSTOM_CAN_SIGNAL_SIGNED);
    uint8_t data[8] = {0x60, 0x57, 0x01, 0, 0, 0, 0, 0x80};
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    TEST_ASSERT_EQUAL_FLOAT(0xABB, Value(0));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, Value(1));
}

void test_motorola_dbc_sawtooth_and_signed_cross_byte()
{
    Message();
    Signal(0, 0, 3, 12, CUSTOM_CAN_SIGNAL_MOTOROLA);
    Signal(1, 0, 19, 12, CUSTOM_CAN_SIGNAL_MOTOROLA | CUSTOM_CAN_SIGNAL_SIGNED);
    uint8_t data[8] = {0x0A, 0xBC, 0x0F, 0xFE};
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    TEST_ASSERT_EQUAL_FLOAT(0xABC, Value(0));
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, Value(1));
}

void test_all_legal_positions_and_widths()
{
    Message();
    uint8_t ones[8] = {255, 255, 255, 255, 255, 255, 255, 255};
    for (uint8_t motorola = 0; motorola < 2; motorola++)
    {
        for (uint8_t start = 0; start < 64; start++)
        {
            for (uint8_t length = 1; length <= 64; length++)
            {
                // Count available bits independently in DBC MSB-first order.
                unsigned available = motorola ? (start % 8 + 1) + (7 - start / 8) * 8 : 64 - start;
                CustomCANSignalConfig config = {0, start, length,
                    static_cast<uint8_t>(CUSTOM_CAN_SIGNAL_ENABLED | CUSTOM_CAN_SIGNAL_SIGNED |
                        (motorola ? CUSTOM_CAN_SIGNAL_MOTOROLA : 0)), 1.0f, 0.0f};
                TEST_ASSERT_EQUAL(length <= available, SetCustomCANSignal(0, config));
                if (length > available)
                {
                 continue;
                }
                ProcessCustomCANFrame(0x180, false, false, ones, 8, 100);
                TEST_ASSERT_EQUAL_FLOAT(-1.0f, Value(0));
            }
        }
    }
}

void test_64_bit_unsigned_and_signed_extremes()
{
    Message();
    Signal(0, 0, 0, 64);
    Signal(1, 0, 0, 64, CUSTOM_CAN_SIGNAL_SIGNED);
    Signal(2, 0, 7, 64, CUSTOM_CAN_SIGNAL_MOTOROLA | CUSTOM_CAN_SIGNAL_SIGNED);
    uint8_t ones[8] = {255, 255, 255, 255, 255, 255, 255, 255};
    ProcessCustomCANFrame(0x180, false, false, ones, 8, 100);
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(UINT64_MAX), Value(0));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, Value(1));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, Value(2));
    uint8_t littleMin[8] = {0, 0, 0, 0, 0, 0, 0, 0x80};
    ProcessCustomCANFrame(0x180, false, false, littleMin, 8, 100);
    TEST_ASSERT_EQUAL_FLOAT(-9223372036854775808.0f, Value(1));
    uint8_t bigMin[8] = {0x80};
    ProcessCustomCANFrame(0x180, false, false, bigMin, 8, 100);
    TEST_ASSERT_EQUAL_FLOAT(-9223372036854775808.0f, Value(2));
}

void test_standard_extended_identity_and_unrelated_frames()
{
    Message(0, 0x180);
    Message(1, 0x180, true);
    Message(2, 0x1ABCDE, true);
    Signal(0, 0, 0, 8);
    Signal(1, 1, 0, 8);
    Signal(2, 2, 0, 8);
    uint8_t data[8] = {42};
    ProcessCustomCANFrame(0x181, false, false, data, 8, 100);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_NOT_RECEIVED, GetCustomCANSignalStatus(0, 100));
    ProcessCustomCANFrame(0x180, true, false, data, 8, 100);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_NOT_RECEIVED, GetCustomCANSignalStatus(0, 100));
    TEST_ASSERT_EQUAL_FLOAT(42.0f, Value(1));
    ProcessCustomCANFrame(0x1ABCDE, true, false, data, 8, 100);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, Value(2));
}

void test_timeout_exact_ceiling_wraparound_and_recovery()
{
    Message();
    Signal(0, 0, 0, 8);
    auto message = CustomCANConfig.Messages[0];
    message.TimeoutMs = 60000;
    TEST_ASSERT_TRUE(SetCustomCANMessage(0, message));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_NOT_RECEIVED, GetCustomCANSignalStatus(0, 0));
    uint8_t data[8] = {42};
    uint32_t received = UINT32_MAX - 100;
    ProcessCustomCANFrame(0x180, false, false, data, 8, received);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, Value(0, received + 59999));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_TIMEOUT, GetCustomCANSignalStatus(0, received + 60000));
    float output = 99.0f;
    TEST_ASSERT_FALSE(ReadCustomCANSignal(0, received + 60000, &output));
    TEST_ASSERT_EQUAL_FLOAT(99.0f, output);
    ProcessCustomCANFrame(0x180, false, false, data, 8, 70000);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, Value(0, 70000));
    UpdateCustomCANTimeouts(130000);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_TIMEOUT, GetCustomCANSignalStatus(0, 70000));
    message.TimeoutMs = 60001;
    TEST_ASSERT_FALSE(SetCustomCANMessage(0, message));
}

void test_timeout_zero_latches_last_value_forever()
{
    Message();
    Signal(0, 0, 0, 8);
    auto message = CustomCANConfig.Messages[0];
    message.TimeoutMs = 0;
    TEST_ASSERT_TRUE(SetCustomCANMessage(0, message));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_NOT_RECEIVED, GetCustomCANSignalStatus(0, 2000000000UL));
    uint8_t data[8] = {42};
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    UpdateCustomCANTimeouts(2000000000UL);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_VALID, GetCustomCANSignalStatus(0, 2000000000UL));
    TEST_ASSERT_EQUAL_FLOAT(42.0f, Value(0, 4000000000UL));
    ProcessCustomCANFrame(0x180, false, true, data, 8, 101);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_INVALID_FRAME, GetCustomCANSignalStatus(0, 101));
}

void test_invalid_frames_invalidate_all_signals_until_recovery()
{
    Message();
    Signal(0, 0, 0, 8);
    Signal(1, 0, 8, 8);
    uint8_t data[8] = {42, 43};
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    ProcessCustomCANFrame(0x180, false, false, data, 1, 101);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_INVALID_FRAME, GetCustomCANSignalStatus(0, 101));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_INVALID_FRAME, GetCustomCANSignalStatus(1, 101));
    ProcessCustomCANFrame(0x180, false, false, data, 8, 102);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, Value(0, 102));
    ProcessCustomCANFrame(0x180, false, true, data, 8, 103);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_INVALID_FRAME, GetCustomCANSignalStatus(0, 103));
    ProcessCustomCANFrame(0x180, false, false, nullptr, 8, 104);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_INVALID_FRAME, GetCustomCANSignalStatus(0, 104));
    ProcessCustomCANFrame(0x180, false, false, data, 9, 105);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_INVALID_FRAME, GetCustomCANSignalStatus(0, 105));
}

void test_reconfiguration_and_sleep_reset_invalidate_cache()
{
    Message();
    Signal(0, 0, 0, 8);
    uint8_t data[8] = {42};
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    auto signal = CustomCANConfig.Signals[0];
    signal.Factor = 2;
    TEST_ASSERT_TRUE(SetCustomCANSignal(0, signal));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_NOT_RECEIVED, GetCustomCANSignalStatus(0, 100));
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    TEST_ASSERT_EQUAL_FLOAT(84.0f, Value(0));
    ResetCustomCANRuntime();
    TEST_ASSERT_EQUAL(CUSTOM_CAN_NOT_RECEIVED, GetCustomCANSignalStatus(0, 100));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, CustomCANConfig.Signals[0].Factor);
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    TEST_ASSERT_TRUE(SetCustomCANParameter(0, 0, 0, 0x181));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_NOT_RECEIVED, GetCustomCANSignalStatus(0, 100));
}

void test_invalid_configuration_transactional_rejection()
{
    Message();
    auto message = CustomCANConfig.Messages[0];
    TEST_ASSERT_FALSE(SetCustomCANMessage(100, message));
    TEST_ASSERT_FALSE(SetCustomCANMessage(1, message));
    message.ID = 0x800;
    TEST_ASSERT_FALSE(SetCustomCANMessage(0, message));
    message.ID = 0x180;
    message.Flags |= 0x80;
    TEST_ASSERT_FALSE(SetCustomCANMessage(0, message));
    Signal(0, 0, 56, 8);
    message = CustomCANConfig.Messages[0];
    message.DLC = 7;
    TEST_ASSERT_FALSE(SetCustomCANMessage(0, message));
    TEST_ASSERT_EQUAL_UINT8(8, CustomCANConfig.Messages[0].DLC);
    auto signal = CustomCANConfig.Signals[0];
    signal.BitLength = 9;
    TEST_ASSERT_FALSE(SetCustomCANSignal(0, signal));
    signal.BitLength = 0;
    TEST_ASSERT_FALSE(SetCustomCANSignal(0, signal));
    signal = CustomCANConfig.Signals[0];
    signal.Factor = NAN;
    TEST_ASSERT_FALSE(SetCustomCANSignal(0, signal));
    signal.Factor = 1;
    signal.Offset = INFINITY;
    TEST_ASSERT_FALSE(SetCustomCANSignal(0, signal));
    signal.Offset = 0;
    signal.MessageIndex = 100;
    TEST_ASSERT_FALSE(SetCustomCANSignal(0, signal));
    TEST_ASSERT_TRUE(ValidateCustomCANConfig());
}

void test_parameter_protocol_and_decode_overflow()
{
    Message();
    Signal(0, 0, 0, 8);
    uint32_t raw = 0;
    float factor = FLT_MAX;
    memcpy(&raw, &factor, sizeof(raw));
    TEST_ASSERT_TRUE(SetCustomCANParameter(1, 0, 4, raw));
    TEST_ASSERT_FALSE(SetCustomCANParameter(0, 0, 1, 65536));
    TEST_ASSERT_FALSE(SetCustomCANParameter(1, 0, 0, 256));
    TEST_ASSERT_FALSE(SetCustomCANParameter(1, 0, 7, 0));
    TEST_ASSERT_FALSE(GetCustomCANParameter(1, 0, 6, 0, &raw));
    TEST_ASSERT_TRUE(GetCustomCANParameter(1, 0, 4, 0, &raw));
    TEST_ASSERT_EQUAL_MEMORY(&factor, &raw, sizeof(raw));
    uint8_t data[8] = {2};
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_DECODE_ERROR, GetCustomCANSignalStatus(0, 100));
    TEST_ASSERT_TRUE(GetCustomCANParameter(1, 0, 7, 100, &raw));
    TEST_ASSERT_EQUAL_UINT32(CUSTOM_CAN_DECODE_ERROR, raw);
}

void test_short_dlc_disabled_sources_and_unrelated_timeout()
{
    auto message = CustomCANConfig.Messages[0];
    message.ID = 0x180;
    message.DLC = 1;
    message.TimeoutMs = 1;
    message.Flags = CUSTOM_CAN_MESSAGE_ENABLED;
    TEST_ASSERT_TRUE(SetCustomCANMessage(0, message));
    Signal(0, 0, 0, 8);
    Signal(1, 0, 7, 8, CUSTOM_CAN_SIGNAL_MOTOROLA);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_NOT_RECEIVED, GetCustomCANSignalStatus(0, 100));
    uint8_t data[8] = {42};
    ProcessCustomCANFrame(0x180, false, false, data, 1, 100);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, Value(0));
    TEST_ASSERT_EQUAL_FLOAT(42.0f, Value(1));
    ProcessCustomCANFrame(0x181, false, false, data, 1, 101);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_TIMEOUT, GetCustomCANSignalStatus(0, 101));
    message.Flags = 0;
    TEST_ASSERT_TRUE(SetCustomCANMessage(0, message));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_DISABLED, GetCustomCANSignalStatus(0, 101));
    ProcessCustomCANFrame(0x180, false, false, data, 1, 101);
    message.Flags = CUSTOM_CAN_MESSAGE_ENABLED;
    TEST_ASSERT_TRUE(SetCustomCANMessage(0, message));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_NOT_RECEIVED, GetCustomCANSignalStatus(0, 101));
}

void test_record_write_is_atomic_and_supports_extended_id_and_relocation()
{
    const uint8_t message[] = {0x67, 0x45, 0x23, 0x01, 0xE8, 0x03, 8, 3, 0};
    TEST_ASSERT_TRUE(ApplyCustomCANWrite(0, 99, CUSTOM_CAN_PARAMETER_RECORD, message, sizeof(message)));
    TEST_ASSERT_EQUAL_HEX32(0x01234567, CustomCANConfig.Messages[99].ID);
    TEST_ASSERT_EQUAL_UINT8(3, CustomCANConfig.Messages[99].Flags);
    TEST_ASSERT_FALSE(ApplyCustomCANWrite(0, 98, CUSTOM_CAN_PARAMETER_RECORD, message, sizeof(message)));
    TEST_ASSERT_EQUAL_UINT32(0, CustomCANConfig.Messages[98].ID);

    Message(0);
    Signal(99, 0, 56, 8);
    auto target = CustomCANConfig.Messages[1];
    target.ID = 0x181;
    target.DLC = 1;
    target.Flags = CUSTOM_CAN_MESSAGE_ENABLED;
    TEST_ASSERT_TRUE(SetCustomCANMessage(1, target));
    const uint8_t signal[] = {1, 0, 8, 1, 0, 0, 0, 0x40, 0, 0, 0x80, 0xBF, 0};
    TEST_ASSERT_TRUE(ApplyCustomCANWrite(1, 99, CUSTOM_CAN_PARAMETER_RECORD, signal, sizeof(signal)));
    TEST_ASSERT_EQUAL_UINT8(1, CustomCANConfig.Signals[99].MessageIndex);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, CustomCANConfig.Signals[99].Factor);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, CustomCANConfig.Signals[99].Offset);
    uint8_t data[] = {5};
    ProcessCustomCANFrame(0x181, false, false, data, 1, 100);
    TEST_ASSERT_EQUAL_FLOAT(9.0f, Value(99));
    CustomCANConfigDirty = false;
    TEST_ASSERT_TRUE(ApplyCustomCANWrite(1, 99, CUSTOM_CAN_PARAMETER_RECORD, signal, sizeof(signal)));
    TEST_ASSERT_FALSE(CustomCANConfigDirty);
    TEST_ASSERT_EQUAL_FLOAT(9.0f, Value(99));
    auto invalid = CustomCANConfig.Signals[99];
    invalid.StartBit = 7;
    TEST_ASSERT_FALSE(ApplyCustomCANWrite(1, 99, CUSTOM_CAN_PARAMETER_RECORD,
                                        reinterpret_cast<const uint8_t *>(&invalid), sizeof(invalid)));
    TEST_ASSERT_EQUAL_MEMORY(signal, &CustomCANConfig.Signals[99], sizeof(signal));
    TEST_ASSERT_FALSE(CustomCANConfigDirty);
}

void test_record_clear_restores_defaults_without_cascading()
{
    Message();
    Signal(0, 0, 0, 8);
    Signal(1, 0, 8, 8);
    const auto other = CustomCANConfig.Signals[1];
    uint8_t data[8] = {42, 43};
    ProcessCustomCANFrame(0x180, false, false, data, 8, 100);
    TEST_ASSERT_TRUE(ApplyCustomCANWrite(1, 0, CUSTOM_CAN_PARAMETER_CLEAR, nullptr, 0));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_DISABLED, GetCustomCANSignalStatus(0, 100));
    TEST_ASSERT_EQUAL_UINT8(1, CustomCANConfig.Signals[0].BitLength);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, CustomCANConfig.Signals[0].Factor);
    TEST_ASSERT_EQUAL_FLOAT(43.0f, Value(1));
    TEST_ASSERT_TRUE(CustomCANConfigDirty);
    TEST_ASSERT_TRUE(ApplyCustomCANWrite(0, 0, CUSTOM_CAN_PARAMETER_CLEAR, nullptr, 0));
    TEST_ASSERT_EQUAL_UINT16(1000, CustomCANConfig.Messages[0].TimeoutMs);
    TEST_ASSERT_EQUAL_UINT8(8, CustomCANConfig.Messages[0].DLC);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_DISABLED, GetCustomCANSignalStatus(1, 100));
    TEST_ASSERT_EQUAL_MEMORY(&other, &CustomCANConfig.Signals[1], sizeof(other));
    CustomCANConfigDirty = false;
    TEST_ASSERT_TRUE(ApplyCustomCANWrite(0, 0, CUSTOM_CAN_PARAMETER_CLEAR, nullptr, 0));
    TEST_ASSERT_FALSE(CustomCANConfigDirty);
}

void test_record_protocol_sizes_and_rejection()
{
    TEST_ASSERT_EQUAL_UINT8(9, GetCustomCANWriteSize(0, CUSTOM_CAN_PARAMETER_RECORD));
    TEST_ASSERT_EQUAL_UINT8(13, GetCustomCANWriteSize(1, CUSTOM_CAN_PARAMETER_RECORD));
    TEST_ASSERT_EQUAL_UINT8(0, GetCustomCANWriteSize(1, CUSTOM_CAN_PARAMETER_CLEAR));
    uint8_t payload[16] = {};
    for (uint8_t type = 0; type < 2; type++)
    {
        uint8_t size = GetCustomCANWriteSize(type, CUSTOM_CAN_PARAMETER_RECORD);
        TEST_ASSERT_FALSE(ApplyCustomCANWrite(type, 0, CUSTOM_CAN_PARAMETER_RECORD, payload, size - 1));
        TEST_ASSERT_FALSE(ApplyCustomCANWrite(type, 0, CUSTOM_CAN_PARAMETER_RECORD, payload, size + 1));
        TEST_ASSERT_FALSE(ApplyCustomCANWrite(type, 0, CUSTOM_CAN_PARAMETER_RECORD, nullptr, size));
        TEST_ASSERT_FALSE(ApplyCustomCANWrite(type, 100, CUSTOM_CAN_PARAMETER_CLEAR, nullptr, 0));
        TEST_ASSERT_FALSE(ApplyCustomCANWrite(type, 0, CUSTOM_CAN_PARAMETER_CLEAR, payload, 1));
        TEST_ASSERT_FALSE(ApplyCustomCANWrite(type, 0, CUSTOM_CAN_PARAMETER_RECORD, payload, size));
    }
    TEST_ASSERT_FALSE(ApplyCustomCANWrite(2, 0, CUSTOM_CAN_PARAMETER_CLEAR, nullptr, 0));
    TEST_ASSERT_FALSE(CustomCANConfigDirty);
    const uint8_t id[] = {0x80, 1, 0, 0};
    TEST_ASSERT_TRUE(ApplyCustomCANWrite(0, 0, 0, id, sizeof(id)));
    TEST_ASSERT_EQUAL_HEX32(0x180, CustomCANConfig.Messages[0].ID);
    TEST_ASSERT_FALSE(ApplyCustomCANWrite(0, 0, 4, id, sizeof(id)));
    TEST_ASSERT_FALSE(ApplyCustomCANWrite(0, 0, 0, id, 3));
}

void test_tx_one_shot_rising_edge_with_constant_payload()
{
    // NMT-start style frame: constant bytes 0x01, 0x15, gated by channel 5.
    TxMessage(0, 0x000, 2, 5);
    TxSignal(0, 0, 0, 8, CUSTOM_CAN_SOURCE_CONSTANT, 0, 1.0f, 1.0f);
    TxSignal(1, 0, 8, 8, CUSTOM_CAN_SOURCE_CONSTANT, 0, 1.0f, 21.0f);
    txChannelValid[5] = true;

    ServiceCustomCANTransmits(0, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(0, txFrameCount);

    txChannelValues[5] = 1.0f;
    ServiceCustomCANTransmits(10, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(1, txFrameCount);
    TEST_ASSERT_EQUAL_HEX32(0x000, txFrames[0].ID);
    TEST_ASSERT_FALSE(txFrames[0].Extended);
    TEST_ASSERT_EQUAL_UINT8(2, txFrames[0].DLC);
    TEST_ASSERT_EQUAL_HEX8(0x01, txFrames[0].Data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x15, txFrames[0].Data[1]);

    // Level stays high: no repeat. Falling then rising edge: one more frame.
    ServiceCustomCANTransmits(20, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(1, txFrameCount);
    txChannelValues[5] = 0.0f;
    ServiceCustomCANTransmits(30, StubReader, StubSender);
    txChannelValues[5] = 2.0f;
    ServiceCustomCANTransmits(40, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(2, txFrameCount);

    // An invalid trigger source reads as condition false; recovery re-fires on the edge.
    txChannelValid[5] = false;
    ServiceCustomCANTransmits(50, StubReader, StubSender);
    txChannelValid[5] = true;
    ServiceCustomCANTransmits(60, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(3, txFrameCount);

    // TX definitions are not receivable and their signals stay unreadable.
    uint8_t data[2] = {1, 0x15};
    ProcessCustomCANFrame(0x000, false, false, data, 2, 70);
    TEST_ASSERT_EQUAL(CUSTOM_CAN_DISABLED, GetCustomCANMessageStatus(0, 70));
    TEST_ASSERT_EQUAL(CUSTOM_CAN_DISABLED, GetCustomCANSignalStatus(0, 70));
    float value = 0.0f;
    TEST_ASSERT_FALSE(ReadCustomCANSignal(0, 70, &value));
}

void test_tx_periodic_schedule_condition_gating_and_values()
{
    // Keypad LED style: 100 ms refresh while trigger channel 1 is true.
    TxMessage(0, 0x215, 8, 1, true, 100);
    TxSignal(0, 0, 0, 1, 2);
    TxSignal(1, 0, 8, 1, 3);
    txChannelValid[1] = txChannelValid[2] = txChannelValid[3] = true;
    txChannelValues[1] = 1.0f;
    txChannelValues[2] = 1.0f;

    ServiceCustomCANTransmits(0, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(1, txFrameCount);
    TEST_ASSERT_EQUAL_HEX8(0x01, txFrames[0].Data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, txFrames[0].Data[1]);

    ServiceCustomCANTransmits(50, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(1, txFrameCount);
    txChannelValues[3] = 5.0f; // clamps to 1 in a 1-bit field
    ServiceCustomCANTransmits(100, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(2, txFrameCount);
    TEST_ASSERT_EQUAL_HEX8(0x01, txFrames[1].Data[1]);

    // Condition false stops transmission; rising edge resumes immediately.
    txChannelValues[1] = 0.0f;
    ServiceCustomCANTransmits(200, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(2, txFrameCount);
    txChannelValues[1] = 1.0f;
    ServiceCustomCANTransmits(230, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(3, txFrameCount);
    ServiceCustomCANTransmits(300, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(3, txFrameCount);
    ServiceCustomCANTransmits(330, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(4, txFrameCount);
}

void test_tx_encode_scaling_signed_motorola_and_clamping()
{
    TxMessage(0, 0x300, 8, CUSTOM_CAN_SOURCE_CONSTANT, true, 100);
    TxSignal(0, 0, 0, 16, 2, 0, 0.25f, -10.0f);                        // (990+10)/0.25 = 0x0FA0
    TxSignal(1, 0, 16, 8, 3, CUSTOM_CAN_SIGNAL_SIGNED);                // -10 -> 0xF6
    TxSignal(2, 0, 27, 12, 4, CUSTOM_CAN_SIGNAL_MOTOROLA);             // 0xABC, DBC sawtooth
    txChannelValid[2] = txChannelValid[3] = txChannelValid[4] = true;
    txChannelValues[2] = 990.0f;
    txChannelValues[3] = -10.0f;
    txChannelValues[4] = 2748.0f;

    ServiceCustomCANTransmits(0, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(1, txFrameCount);
    TEST_ASSERT_EQUAL_HEX8(0xA0, txFrames[0].Data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x0F, txFrames[0].Data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xF6, txFrames[0].Data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x0A, txFrames[0].Data[3]);
    TEST_ASSERT_EQUAL_HEX8(0xBC, txFrames[0].Data[4]);

    // Round-trip: an identical RX definition decodes the captured frame to the source values.
    auto rx = CustomCANConfig.Messages[1];
    rx.ID = 0x301;
    rx.DLC = 8;
    rx.Flags = CUSTOM_CAN_MESSAGE_ENABLED;
    TEST_ASSERT_TRUE(SetCustomCANMessage(1, rx));
    Signal(10, 1, 0, 16, 0, 0.25f, -10.0f);
    Signal(11, 1, 16, 8, CUSTOM_CAN_SIGNAL_SIGNED);
    Signal(12, 1, 27, 12, CUSTOM_CAN_SIGNAL_MOTOROLA);
    ProcessCustomCANFrame(0x301, false, false, txFrames[0].Data, 8, 100);
    TEST_ASSERT_EQUAL_FLOAT(990.0f, Value(10));
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, Value(11));
    TEST_ASSERT_EQUAL_FLOAT(2748.0f, Value(12));

    // Clamping at field extremes.
    txChannelValues[3] = 500.0f;   // signed 8-bit -> 127
    txChannelValues[4] = -5.0f;    // unsigned -> 0
    ServiceCustomCANTransmits(100, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(2, txFrameCount);
    TEST_ASSERT_EQUAL_HEX8(0x7F, txFrames[1].Data[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, txFrames[1].Data[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, txFrames[1].Data[4] & 0xF0);
}

void test_tx_validation_rules()
{
    // Periodic needs a non-zero period; TX_PERIODIC requires TX.
    CustomCANMessageConfig message = {0x100, 0, 8,
        CUSTOM_CAN_MESSAGE_ENABLED | CUSTOM_CAN_MESSAGE_TX | CUSTOM_CAN_MESSAGE_TX_PERIODIC, 0};
    TEST_ASSERT_FALSE(SetCustomCANMessage(0, message));
    message.TimeoutMs = 100;
    TEST_ASSERT_TRUE(SetCustomCANMessage(0, message));
    message.Flags = CUSTOM_CAN_MESSAGE_ENABLED | CUSTOM_CAN_MESSAGE_TX_PERIODIC;
    message.TimeoutMs = 1000;
    TEST_ASSERT_FALSE(SetCustomCANMessage(0, message));

    // Trigger must be a channel or the constant source.
    message = {0x100, 0, 8, CUSTOM_CAN_MESSAGE_ENABLED | CUSTOM_CAN_MESSAGE_TX,
               CUSTOM_CAN_TX_CHANNEL_SOURCES};
    TEST_ASSERT_FALSE(SetCustomCANMessage(0, message));
    message.TxTrigger = CUSTOM_CAN_SOURCE_CONSTANT;
    TEST_ASSERT_TRUE(SetCustomCANMessage(0, message));

    // DLC 0 is TX-only, and such frames cannot carry signals.
    message = {0x080, 0, 0, CUSTOM_CAN_MESSAGE_ENABLED | CUSTOM_CAN_MESSAGE_TX, 0};
    TEST_ASSERT_TRUE(SetCustomCANMessage(1, message));
    CustomCANSignalConfig orphan = {1, 0, 1, CUSTOM_CAN_SIGNAL_ENABLED, 1.0f, 0.0f, 0};
    TEST_ASSERT_FALSE(SetCustomCANSignal(0, orphan));
    message.Flags = CUSTOM_CAN_MESSAGE_ENABLED;
    TEST_ASSERT_FALSE(SetCustomCANMessage(1, message));

    // Duplicate IDs: TX+TX and TX+RX may share, RX+RX may not.
    TxMessage(2, 0x215, 8, 0);
    TxMessage(3, 0x215, 8, 1);
    Message(4, 0x215);
    CustomCANMessageConfig duplicate = {0x215, 1000, 8, CUSTOM_CAN_MESSAGE_ENABLED, 0};
    TEST_ASSERT_FALSE(SetCustomCANMessage(5, duplicate));

    // A non-constant TX signal needs a usable encode factor.
    CustomCANSignalConfig zeroFactor = {2, 0, 8, CUSTOM_CAN_SIGNAL_ENABLED, 0.0f, 0.0f, 0};
    TEST_ASSERT_FALSE(SetCustomCANSignal(1, zeroFactor));
    zeroFactor.ValueSource = CUSTOM_CAN_SOURCE_CONSTANT;
    TEST_ASSERT_TRUE(SetCustomCANSignal(1, zeroFactor));
    zeroFactor.ValueSource = CUSTOM_CAN_TX_CHANNEL_SOURCES;
    TEST_ASSERT_FALSE(SetCustomCANSignal(1, zeroFactor));

    // Flipping an RX message with a factor-0 signal to TX is rejected transactionally.
    Message(6, 0x400);
    CustomCANSignalConfig rxZero = {6, 0, 8, CUSTOM_CAN_SIGNAL_ENABLED, 0.0f, 0.0f, 0};
    TEST_ASSERT_TRUE(SetCustomCANSignal(2, rxZero));
    auto flipped = CustomCANConfig.Messages[6];
    flipped.Flags |= CUSTOM_CAN_MESSAGE_TX;
    TEST_ASSERT_FALSE(SetCustomCANMessage(6, flipped));
    TEST_ASSERT_TRUE(ValidateCustomCANConfig());
}

void test_tx_pending_retries_after_send_or_source_failure()
{
    TxMessage(0, 0x123, 1, 0);
    TxSignal(0, 0, 0, 8, 1);
    txChannelValid[0] = true;
    txChannelValues[0] = 1.0f;

    // Value source unavailable: the one-shot stays pending until it recovers.
    ServiceCustomCANTransmits(0, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(0, txFrameCount);
    txChannelValid[1] = true;
    txChannelValues[1] = 77.0f;
    ServiceCustomCANTransmits(10, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(1, txFrameCount);
    TEST_ASSERT_EQUAL_HEX8(77, txFrames[0].Data[0]);

    // Full transmit queue: retried on the next service call without a new edge.
    txChannelValues[0] = 0.0f;
    ServiceCustomCANTransmits(20, StubReader, StubSender);
    txSendResult = false;
    txChannelValues[0] = 1.0f;
    ServiceCustomCANTransmits(30, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(1, txFrameCount);
    txSendResult = true;
    ServiceCustomCANTransmits(40, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(2, txFrameCount);

    // Null hooks are a no-op.
    ServiceCustomCANTransmits(50, nullptr, StubSender);
    ServiceCustomCANTransmits(50, StubReader, nullptr);
    TEST_ASSERT_EQUAL_UINT8(2, txFrameCount);
}

void test_tx_parameters_and_reconfiguration_resets_state()
{
    TxMessage(0, 0x123, 1, 0);
    TxSignal(0, 0, 0, 8, CUSTOM_CAN_SOURCE_CONSTANT, 0, 1.0f, 9.0f);
    uint32_t value = 0;
    TEST_ASSERT_TRUE(GetCustomCANParameter(0, 0, 5, 0, &value));
    TEST_ASSERT_EQUAL_UINT32(0, value);
    TEST_ASSERT_TRUE(SetCustomCANParameter(0, 0, 5, CUSTOM_CAN_SOURCE_CONSTANT));
    TEST_ASSERT_FALSE(SetCustomCANParameter(0, 0, 5, CUSTOM_CAN_TX_CHANNEL_SOURCES));
    TEST_ASSERT_TRUE(GetCustomCANParameter(0, 0, 5, 0, &value));
    TEST_ASSERT_EQUAL_UINT32(CUSTOM_CAN_SOURCE_CONSTANT, value);
    TEST_ASSERT_TRUE(GetCustomCANParameter(1, 0, 8, 0, &value));
    TEST_ASSERT_EQUAL_UINT32(CUSTOM_CAN_SOURCE_CONSTANT, value);
    TEST_ASSERT_TRUE(SetCustomCANParameter(1, 0, 8, 3));
    TEST_ASSERT_FALSE(SetCustomCANParameter(1, 0, 8, 256));

    // The constant trigger fires once; reconfiguring the message re-arms the edge detector.
    TEST_ASSERT_TRUE(SetCustomCANParameter(1, 0, 8, CUSTOM_CAN_SOURCE_CONSTANT));
    ServiceCustomCANTransmits(0, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(1, txFrameCount);
    TEST_ASSERT_EQUAL_HEX8(9, txFrames[0].Data[0]);
    ServiceCustomCANTransmits(10, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(1, txFrameCount);
    TEST_ASSERT_TRUE(SetCustomCANParameter(0, 0, 0, 0x124));
    ServiceCustomCANTransmits(20, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(2, txFrameCount);
    TEST_ASSERT_EQUAL_HEX32(0x124, txFrames[1].ID);

    // Sleep/CAN reinitialisation re-arms TX state as well.
    ResetCustomCANRuntime();
    ServiceCustomCANTransmits(30, StubReader, StubSender);
    TEST_ASSERT_EQUAL_UINT8(3, txFrameCount);
}

static int RunTests()
{
    UNITY_BEGIN();
    RUN_TEST(test_layout_defaults_and_capacity);
    RUN_TEST(test_shared_frame_unsigned_signed_and_scaling);
    RUN_TEST(test_little_endian_cross_byte_and_bit_boundaries);
    RUN_TEST(test_motorola_dbc_sawtooth_and_signed_cross_byte);
    RUN_TEST(test_all_legal_positions_and_widths);
    RUN_TEST(test_64_bit_unsigned_and_signed_extremes);
    RUN_TEST(test_standard_extended_identity_and_unrelated_frames);
    RUN_TEST(test_timeout_exact_ceiling_wraparound_and_recovery);
    RUN_TEST(test_timeout_zero_latches_last_value_forever);
    RUN_TEST(test_invalid_frames_invalidate_all_signals_until_recovery);
    RUN_TEST(test_reconfiguration_and_sleep_reset_invalidate_cache);
    RUN_TEST(test_invalid_configuration_transactional_rejection);
    RUN_TEST(test_parameter_protocol_and_decode_overflow);
    RUN_TEST(test_short_dlc_disabled_sources_and_unrelated_timeout);
    RUN_TEST(test_record_write_is_atomic_and_supports_extended_id_and_relocation);
    RUN_TEST(test_record_clear_restores_defaults_without_cascading);
    RUN_TEST(test_record_protocol_sizes_and_rejection);
    RUN_TEST(test_tx_one_shot_rising_edge_with_constant_payload);
    RUN_TEST(test_tx_periodic_schedule_condition_gating_and_values);
    RUN_TEST(test_tx_encode_scaling_signed_motorola_and_clamping);
    RUN_TEST(test_tx_validation_rules);
    RUN_TEST(test_tx_pending_retries_after_send_or_source_failure);
    RUN_TEST(test_tx_parameters_and_reconfiguration_resets_state);
    return UNITY_END();
}

#ifdef ARDUINO
void setup()
{
    delay(1000);
    RunTests();
}
void loop() {}
#else
int main()
{
    return RunTests();
}
#endif
