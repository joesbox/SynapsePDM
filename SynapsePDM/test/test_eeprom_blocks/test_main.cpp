#include <unity.h>
#include "../../src/EEPROMBlocks.h"
#include "../../src/CustomCAN.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

struct DestructiveEEPROM
{
    uint8_t Bytes[8192];
    uint16_t Writes;
    uint16_t Waits;

    void EepromRead(uint16_t address, uint8_t length, uint8_t *data)
    {
        TEST_ASSERT_TRUE(address + length <= sizeof(Bytes));
        memcpy(data, Bytes + address, length);
    }

    void EepromWrite(uint16_t address, uint8_t length, uint8_t *data)
    {
        TEST_ASSERT_TRUE(length > 0 && length <= 32);
        TEST_ASSERT_TRUE(address + length <= sizeof(Bytes));
        TEST_ASSERT_EQUAL_UINT16(address / 32, (address + length - 1) / 32);
        memcpy(Bytes + address, data, length);
        memset(data, 0xFF, length);
        Writes++;
    }

    void EepromWaitEndWriteOperation() { Waits++; }
};

static DestructiveEEPROM device;
static CustomCANConfiguration config;
static uint8_t original[sizeof(config)];

void setUp()
{
    memset(&device, 0, sizeof(device));
    memset(device.Bytes, 0xA5, sizeof(device.Bytes));
    for (uint16_t i = 0; i < sizeof(original); i++) original[i] = static_cast<uint8_t>(i * 7);
    memcpy(&config, original, sizeof(config));
}

void tearDown() {}

void test_unaligned_can_bank_preserves_ram_and_page_boundaries()
{
    const uint16_t start = 3668 + 5;
    WriteEEPROMPages<32>(device, start, reinterpret_cast<const uint8_t *>(&config), sizeof(config));
    TEST_ASSERT_EQUAL_MEMORY(original, &config, sizeof(config));
    TEST_ASSERT_EQUAL_MEMORY(original, device.Bytes + start, sizeof(config));
    uint8_t before = device.Bytes[start - 1];
    uint8_t after = device.Bytes[start + sizeof(config)];
    TEST_ASSERT_EQUAL_UINT8(0xA5, before);
    TEST_ASSERT_EQUAL_UINT8(0xA5, after);
    TEST_ASSERT_EQUAL_UINT16(device.Writes, device.Waits);
    TEST_ASSERT_TRUE(device.Writes > 1);
}

void test_header_and_checksum_survive_destructive_driver()
{
    const uint8_t header[] = {0x31, 0x4E, 0x41, 0x43, 1};
    const uint8_t checksum[] = {0x12, 0x34, 0x56, 0x78};
    WriteEEPROMPages<32>(device, 3668, header, sizeof(header));
    WriteEEPROMPages<32>(device, 5673, checksum, sizeof(checksum));
    TEST_ASSERT_EQUAL_MEMORY(header, device.Bytes + 3668, sizeof(header));
    TEST_ASSERT_EQUAL_MEMORY(checksum, device.Bytes + 5673, sizeof(checksum));
    TEST_ASSERT_EQUAL_HEX8(0x31, header[0]);
    TEST_ASSERT_EQUAL_HEX8(0x12, checksum[0]);
}

void test_aligned_and_empty_writes()
{
    WriteEEPROMPages<32>(device, 32, original, 64);
    TEST_ASSERT_EQUAL_UINT16(2, device.Writes);
    TEST_ASSERT_EQUAL_MEMORY(original, device.Bytes + 32, 64);
    WriteEEPROMPages<32>(device, 0, nullptr, 0);
    TEST_ASSERT_EQUAL_UINT16(2, device.Writes);
}

static void UpdateBank()
{
    const uint8_t header[] = {0x31, 0x4E, 0x41, 0x43, 1};
    const uint8_t checksum[] = {0x12, 0x34, 0x56, 0x78};
    WriteEEPROMPages<32>(device, 3668, header, sizeof(header), true);
    WriteEEPROMPages<32>(device, 3673, reinterpret_cast<const uint8_t *>(&config), sizeof(config), true);
    WriteEEPROMPages<32>(device, 5673, checksum, sizeof(checksum), true);
}

void test_unchanged_bank_requires_no_write_cycles()
{
    UpdateBank();
    TEST_ASSERT_EQUAL_UINT16(66, device.Writes);
    device.Writes = device.Waits = 0;
    UpdateBank();
    WriteEEPROMPages<32>(device, 0, nullptr, 0, true);
    TEST_ASSERT_EQUAL_UINT16(0, device.Writes);
    TEST_ASSERT_EQUAL_UINT16(0, device.Waits);
    TEST_ASSERT_EQUAL_MEMORY(original, &config, sizeof(config));
    TEST_ASSERT_EQUAL_MEMORY(original, device.Bytes + 3673, sizeof(config));
    TEST_ASSERT_EQUAL_HEX8(0xA5, device.Bytes[3667]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, device.Bytes[5677]);
}

void test_signal_edit_writes_only_one_or_two_pages_plus_checksum()
{
    UpdateBank();
    for (uint8_t index = 0; index < MAX_CUSTOM_CAN_SIGNALS; index++)
    {
        device.Writes = device.Waits = 0;
        auto *bytes = reinterpret_cast<uint8_t *>(&config.Signals[index]);
        for (uint8_t i = 0; i < sizeof(CustomCANSignalConfig); i++) bytes[i] ^= 0xFF;
        // Simulate a different CRC after each edit.
        device.Bytes[5673] ^= 0xFF;
        UpdateBank();
        uint16_t address = 3673 + sizeof(config.Messages) + index * sizeof(CustomCANSignalConfig);
        uint16_t pages = address / 32 == (address + sizeof(CustomCANSignalConfig) - 1) / 32 ? 1 : 2;
        TEST_ASSERT_EQUAL_UINT16(pages + 1, device.Writes);
        TEST_ASSERT_EQUAL_UINT16(device.Writes, device.Waits);
        TEST_ASSERT_EQUAL_MEMORY(&config, device.Bytes + 3673, sizeof(config));
    }
}

void test_update_repairs_corruption_even_outside_edited_record()
{
    UpdateBank();
    device.Writes = device.Waits = 0;
    device.Bytes[3668] ^= 1;
    device.Bytes[5000] ^= 1;
    device.Bytes[5676] ^= 1;
    UpdateBank();
    TEST_ASSERT_EQUAL_UINT16(3, device.Writes);
    TEST_ASSERT_EQUAL_MEMORY(&config, device.Bytes + 3673, sizeof(config));
    device.Writes = 0;
    UpdateBank();
    TEST_ASSERT_EQUAL_UINT16(0, device.Writes);
}

static int RunTests()
{
    UNITY_BEGIN();
    RUN_TEST(test_unaligned_can_bank_preserves_ram_and_page_boundaries);
    RUN_TEST(test_header_and_checksum_survive_destructive_driver);
    RUN_TEST(test_aligned_and_empty_writes);
    RUN_TEST(test_unchanged_bank_requires_no_write_cycles);
    RUN_TEST(test_signal_edit_writes_only_one_or_two_pages_plus_checksum);
    RUN_TEST(test_update_repairs_corruption_even_outside_edited_record);
    return UNITY_END();
}

#ifdef ARDUINO
void setup() { delay(1000); RunTests(); }
void loop() {}
#else
int main() { return RunTests(); }
#endif
