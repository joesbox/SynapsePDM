#ifndef EEPROMBlocks_H
#define EEPROMBlocks_H

#include <stdint.h>
#include <string.h>

template <uint8_t PageSize, typename Device>
void WriteEEPROMPages(Device &device, uint16_t address, const uint8_t *source, uint16_t length,
                      bool skipUnchanged = false)
{
    static_assert(PageSize > 0, "EEPROM page size must be nonzero");
    uint8_t page[PageSize];
    while (length > 0)
    {
        uint8_t available = static_cast<uint8_t>(PageSize - address % PageSize);
        uint8_t count = length < available ? static_cast<uint8_t>(length) : available;
        if (skipUnchanged)
        {
            device.EepromRead(address, count, page);
        }
        if (!skipUnchanged || memcmp(page, source, count) != 0)
        {
            // M95640R writes use an in-place SPI transfer that destroys this buffer.
            memcpy(page, source, count);
            device.EepromWrite(address, count, page);
            device.EepromWaitEndWriteOperation();
        }
        address += count;
        source += count;
        length -= count;
    }
}

template <uint8_t PageSize, typename Device>
void EraseEEPROMOutsideRange(Device &device, uint16_t totalBytes,
                            uint16_t preservedStart, uint16_t preservedLength)
{
    uint8_t blank[PageSize];
    memset(blank, 0xFF, sizeof(blank));
    for (uint16_t address = 0; address < totalBytes;)
    {
        if (address == preservedStart)
        {
            address += preservedLength;
            continue;
        }
        uint16_t end = address < preservedStart ? preservedStart : totalBytes;
        uint16_t count = end - address;
        if (count > sizeof(blank)) count = sizeof(blank);
        WriteEEPROMPages<PageSize>(device, address, blank, count);
        address += count;
    }
}

#endif
