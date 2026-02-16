/*  ConfigStorage.cpp Functions and variables for EEPROM storage and SD data logging.
    Copyright (c) 2023 Joe Mann.  All right reserved.

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*/

#include "Storage.h"

uint16_t bufferIndex = 0;
StorageConfigUnion StorageConfigData;
StorageParameters StorageParams;
uint16_t EEPROMindex;
File dataFile;
char fileName[24];
char dateTimeStamp[23];
uint32_t BytesStored;
bool UndervoltageLatch;
bool StorageCRCValid;
bool AnalogueCRCValid;
bool SDFileOpen = false; // Track whether SD file is currently open

CircularBuffer<String, 10> logs;

uint32_t lineCount;

M95640R EEPROMext(&SPI_2, CS1);

const char systemHeader[] = "Date,Time,System Temp,System Voltage,System Current,Error Flags,IMU Accel X,IMU Accel Y,IMU Accel Z,IMU Gyro X,IMU Gyro Y,IMU Gyro Z,Lat,Lon,Alt,Speed,Accuracy,";
const char channelHeader[] = "Channel Type,Enabled,Current Value,Current Threshold High,Current Threshold Low,Multi-Channel,Group Number,Channel Error Flags";

long startMillis;
long endMillis;

// #define DEBUG

void SaveChannelConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);

    EEPROMindex = 0;

    // Copy current channel info to storage structure
    memcpy(&ChannelConfigData.data, &Channels, sizeof(Channels));

    // Calculate CRC
    uint32_t checksum = CRC32::calculate(
        ChannelConfigData.dataBytes,
        sizeof(ChannelConfigData.dataBytes));

    const uint8_t *src = ChannelConfigData.dataBytes;
    size_t bytesRemaining = sizeof(ChannelConfigData.dataBytes);

    uint8_t pageBuf[EEPROM_PAGE_SIZE];

    while (bytesRemaining > 0)
    {
        // How many bytes until the next page boundary?
        uint8_t pageOffset = EEPROMindex % EEPROM_PAGE_SIZE;
        uint8_t spaceInPage = EEPROM_PAGE_SIZE - pageOffset;

        uint8_t writeLen = (bytesRemaining < spaceInPage)
                               ? bytesRemaining
                               : spaceInPage;

        memcpy(pageBuf, src, writeLen);

        EEPROMext.EepromWrite(EEPROMindex, writeLen, pageBuf);
        EEPROMext.EepromWaitEndWriteOperation();

        EEPROMindex += writeLen;
        src += writeLen;
        bytesRemaining -= writeLen;
    }

    // Write CRC (4 bytes) — will naturally page-align if needed
    uint8_t crcBuf[4];
    crcBuf[0] = (checksum >> 24) & 0xFF;
    crcBuf[1] = (checksum >> 16) & 0xFF;
    crcBuf[2] = (checksum >> 8) & 0xFF;
    crcBuf[3] = checksum & 0xFF;

    EEPROMext.EepromWrite(EEPROMindex, sizeof(crcBuf), crcBuf);
    EEPROMext.EepromWaitEndWriteOperation();

#ifdef DEBUG
    Serial.print("Channel Checksum written: ");
    Serial.print(checksum, HEX);
    Serial.print(", final index: ");
    Serial.println(EEPROMindex + sizeof(crcBuf));
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif

    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();
}

bool LoadChannelConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);

    bool validCRC = false;
    EEPROMindex = 0;

    uint8_t int32Buf[4];

    // Read channel config data in 32-byte chunks
    const uint16_t totalSize = sizeof(ChannelConfigData.dataBytes);
    uint16_t bytesRemaining = totalSize;
    uint16_t addr = EEPROMindex;
    uint8_t *dst = ChannelConfigData.dataBytes;

    while (bytesRemaining > 0)
    {
        uint8_t chunk =
            (bytesRemaining > 32) ? 32 : bytesRemaining;

        EEPROMext.EepromRead(addr, chunk, dst);

        addr += chunk;
        dst += chunk;
        bytesRemaining -= chunk;
    }

    EEPROMindex += totalSize;

    // Read stored CRC
    EEPROMext.EepromRead(EEPROMindex, sizeof(int32Buf), int32Buf);

    uint32_t result =
        (uint32_t(int32Buf[0]) << 24) |
        (uint32_t(int32Buf[1]) << 16) |
        (uint32_t(int32Buf[2]) << 8) |
        (uint32_t(int32Buf[3]));

#ifdef DEBUG
    Serial.print("Channel Checksum read: ");
    Serial.print(result, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif

    // Calculate CRC
    uint32_t checksum = CRC32::calculate(
        ChannelConfigData.dataBytes,
        sizeof(ChannelConfigData.dataBytes));

    // Validate CRC
    if (result == checksum)
    {
        validCRC = true;
        memcpy(&Channels, &ChannelConfigData.data, sizeof(Channels));
    }

    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();

    return validCRC;
}

void SaveSystemConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);

    // System info comes straight after channel info + CRC
    EEPROMindex = sizeof(ChannelConfigData.dataBytes) + sizeof(uint32_t);

#ifdef DEBUG
    Serial.print("System write start index: ");
    Serial.println(EEPROMindex);
#endif

    // Clear storage structure
    memset(&SystemConfigData, 0, sizeof(SystemConfigData));

    // Copy current system info to storage structure
    memcpy(&SystemConfigData.data, &SystemParams, sizeof(SystemParams));

    // Calculate CRC
    uint32_t checksum = CRC32::calculate(
        SystemConfigData.dataBytes,
        sizeof(SystemConfigData.dataBytes));

    const uint8_t *src = SystemConfigData.dataBytes;
    size_t bytesRemaining = sizeof(SystemConfigData.dataBytes);

    uint8_t pageBuf[EEPROM_PAGE_SIZE];

    while (bytesRemaining > 0)
    {
        uint8_t pageOffset = EEPROMindex % EEPROM_PAGE_SIZE;
        uint8_t spaceInPage = EEPROM_PAGE_SIZE - pageOffset;

        uint8_t writeLen = (bytesRemaining < spaceInPage)
                               ? bytesRemaining
                               : spaceInPage;

        memcpy(pageBuf, src, writeLen);

        EEPROMext.EepromWrite(EEPROMindex, writeLen, pageBuf);
        EEPROMext.EepromWaitEndWriteOperation();

        EEPROMindex += writeLen;
        src += writeLen;
        bytesRemaining -= writeLen;
    }

    // Write CRC (4 bytes)
    uint8_t crcBuf[4];
    crcBuf[0] = (checksum >> 24) & 0xFF;
    crcBuf[1] = (checksum >> 16) & 0xFF;
    crcBuf[2] = (checksum >> 8) & 0xFF;
    crcBuf[3] = checksum & 0xFF;

    EEPROMext.EepromWrite(EEPROMindex, sizeof(crcBuf), crcBuf);
    EEPROMext.EepromWaitEndWriteOperation();

#ifdef DEBUG
    Serial.print("System Checksum written: ");
    Serial.print(checksum, HEX);
    Serial.print(", final index: ");
    Serial.println(EEPROMindex + sizeof(crcBuf));
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
    Serial.print("SystemParams size: ");
    Serial.println(sizeof(SystemParams));
    Serial.print("First 4 bytes written: ");
    Serial.print(SystemConfigData.dataBytes[0], HEX);
    Serial.print(" ");
    Serial.print(SystemConfigData.dataBytes[1], HEX);
    Serial.print(" ");
    Serial.print(SystemConfigData.dataBytes[2], HEX);
    Serial.print(" ");
    Serial.println(SystemConfigData.dataBytes[3], HEX);
#endif

    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();
}

bool LoadSystemConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);

    bool validCRC = false;

    // System config follows channel config + CRC
    EEPROMindex = sizeof(ChannelConfigData.dataBytes) + sizeof(uint32_t);

#ifdef DEBUG
    Serial.print("System read start index: ");
    Serial.println(EEPROMindex);
    Serial.print("sizeof(ChannelConfigData.data): ");
    Serial.println(sizeof(ChannelConfigData.dataBytes));
#endif

    uint8_t int32Buf[4];

    // Read system config data in 32-byte chunks
    const uint16_t totalSize = sizeof(SystemConfigData.dataBytes);
    uint16_t bytesRemaining = totalSize;
    uint16_t addr = EEPROMindex;
    uint8_t *dst = SystemConfigData.dataBytes;

    while (bytesRemaining > 0)
    {
        uint8_t chunk =
            (bytesRemaining > 32) ? 32 : bytesRemaining;

        EEPROMext.EepromRead(addr, chunk, dst);

        addr += chunk;
        dst += chunk;
        bytesRemaining -= chunk;
    }

    EEPROMindex += totalSize;

    // Read stored CRC
    EEPROMext.EepromRead(EEPROMindex, sizeof(int32Buf), int32Buf);

    uint32_t result =
        (uint32_t(int32Buf[0]) << 24) |
        (uint32_t(int32Buf[1]) << 16) |
        (uint32_t(int32Buf[2]) << 8) |
        (uint32_t(int32Buf[3]));

#ifdef DEBUG
    Serial.print("System checksum read: ");
    Serial.print(result, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif

    // Calculate CRC
    uint32_t checksum = CRC32::calculate(
        SystemConfigData.dataBytes,
        sizeof(SystemConfigData.dataBytes));

#ifdef DEBUG
    Serial.print("System checksum calculated: ");
    Serial.println(checksum, HEX);
    Serial.print("SystemParams size: ");
    Serial.println(sizeof(SystemParams));
    Serial.print("First 4 bytes read: ");
    Serial.print(SystemConfigData.dataBytes[0], HEX);
    Serial.print(" ");
    Serial.print(SystemConfigData.dataBytes[1], HEX);
    Serial.print(" ");
    Serial.print(SystemConfigData.dataBytes[2], HEX);
    Serial.print(" ");
    Serial.println(SystemConfigData.dataBytes[3], HEX);
#endif

    // Validate CRC
    if (result == checksum)
    {
        validCRC = true;
        memcpy(&SystemParams, &SystemConfigData.data, sizeof(SystemParams));
    }

    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();

    return validCRC;
}

void SaveStorageConfig()
{
    SPI_2.begin();

    if (StorageParams.LogFrequency == 0)
    {
        StorageParams.LogFrequency = DEFAULT_LOG_FREQUENCY;
    }

    if (StorageParams.MaxLogLength == 0)
    {
        StorageParams.MaxLogLength = DEFAULT_LOG_LINES;
    }

    EEPROMext.begin(EEPROM_SPI_SPEED);

    // Storage config follows channel + CRC + system + CRC
    EEPROMindex =
        sizeof(ChannelConfigData.dataBytes) + sizeof(uint32_t) +
        sizeof(SystemConfigData.dataBytes) + sizeof(uint32_t);

    // Copy current storage info to storage structure
    memcpy(&StorageConfigData.data, &StorageParams, sizeof(StorageParameters));

    // Calculate CRC
    uint32_t checksum = CRC32::calculate(
        StorageConfigData.dataBytes,
        sizeof(StorageConfigData.dataBytes));

    uint8_t int32Buf[4] =
        {
            (uint8_t)(checksum >> 24),
            (uint8_t)(checksum >> 16),
            (uint8_t)(checksum >> 8),
            (uint8_t)(checksum)};

    // Write storage config in 32-byte page-safe chunks
    uint16_t addr = EEPROMindex;
    const uint8_t *src = StorageConfigData.dataBytes;
    uint16_t bytesRemaining = sizeof(StorageConfigData.dataBytes);

    while (bytesRemaining > 0)
    {
        uint8_t pageOffset = addr % 32;
        uint8_t spaceInPage = 32 - pageOffset;
        uint8_t writeLen =
            (bytesRemaining < spaceInPage) ? bytesRemaining : spaceInPage;

        EEPROMext.EepromWrite(addr, writeLen, (uint8_t *)src);
        EEPROMext.EepromWaitEndWriteOperation();

        addr += writeLen;
        src += writeLen;
        bytesRemaining -= writeLen;
    }

    EEPROMindex += sizeof(StorageConfigData.dataBytes);

#ifdef DEBUG
    Serial.print("Storage Checksum written: ");
    Serial.print(checksum, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif

    // Write CRC
    EEPROMext.EepromWrite(EEPROMindex, sizeof(int32Buf), int32Buf);
    EEPROMext.EepromWaitEndWriteOperation();

    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();
}

bool LoadStorageConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);

    bool validCRC = false;

    // Storage config follows channel + CRC + system + CRC
    EEPROMindex =
        sizeof(ChannelConfigData.dataBytes) + sizeof(uint32_t) +
        sizeof(SystemConfigData.dataBytes) + sizeof(uint32_t);

    uint8_t int32Buf[4];

    // Read storage config data in 32-byte chunks
    const uint16_t totalSize = sizeof(StorageConfigData.dataBytes);
    uint16_t bytesRemaining = totalSize;
    uint16_t addr = EEPROMindex;
    uint8_t *dst = StorageConfigData.dataBytes;

    while (bytesRemaining > 0)
    {
        uint8_t chunk =
            (bytesRemaining > 32) ? 32 : bytesRemaining;

        EEPROMext.EepromRead(addr, chunk, dst);

        addr += chunk;
        dst += chunk;
        bytesRemaining -= chunk;
    }

    EEPROMindex += totalSize;

    // Read stored CRC
    EEPROMext.EepromRead(EEPROMindex, sizeof(int32Buf), int32Buf);

    uint32_t result =
        (uint32_t(int32Buf[0]) << 24) |
        (uint32_t(int32Buf[1]) << 16) |
        (uint32_t(int32Buf[2]) << 8) |
        (uint32_t(int32Buf[3]));

#ifdef DEBUG
    Serial.print("Storage checksum read: ");
    Serial.print(result, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif

    // Calculate CRC
    uint32_t checksum = CRC32::calculate(
        StorageConfigData.dataBytes,
        sizeof(StorageConfigData.dataBytes));

    // Validate CRC
    if (result == checksum)
    {
        validCRC = true;

        memcpy(&StorageParams, &StorageConfigData.data, sizeof(StorageParams));

        // Rebuild circular buffer of log files
        for (int i = 0; i < 10; i++)
        {
            if (strlen(StorageParams.LogFileNames[i]) != 0)
            {
                logs.unshift(StorageParams.LogFileNames[i]);
            }
        }
    }

    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();

    return validCRC;
}

void SaveAnalogueConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);

    // Analogue config follows channel + CRC + system + CRC + storage + CRC
    EEPROMindex =
        sizeof(ChannelConfigData.dataBytes) + sizeof(uint32_t) +
        sizeof(SystemConfigData.dataBytes) + sizeof(uint32_t) +
        sizeof(StorageConfigData.dataBytes) + sizeof(uint32_t);

    // Copy current analogue input info to storage structure
    memcpy(&AnalogueConfigData.data, &AnalogueIns, sizeof(AnalogueIns));

    // Calculate CRC
    uint32_t checksum = CRC32::calculate(
        AnalogueConfigData.dataBytes,
        sizeof(AnalogueConfigData.dataBytes));

    uint8_t int32Buf[4] =
        {
            (uint8_t)(checksum >> 24),
            (uint8_t)(checksum >> 16),
            (uint8_t)(checksum >> 8),
            (uint8_t)(checksum)};

    // Write analogue config data in 32-byte page-safe chunks
    uint16_t addr = EEPROMindex;
    const uint8_t *src = AnalogueConfigData.dataBytes;
    uint16_t bytesRemaining = sizeof(AnalogueConfigData.dataBytes);

    while (bytesRemaining > 0)
    {
        uint8_t pageOffset = addr % 32;
        uint8_t spaceInPage = 32 - pageOffset;
        uint8_t writeLen =
            (bytesRemaining < spaceInPage) ? bytesRemaining : spaceInPage;

        EEPROMext.EepromWrite(addr, writeLen, (uint8_t *)src);
        EEPROMext.EepromWaitEndWriteOperation();

        addr += writeLen;
        src += writeLen;
        bytesRemaining -= writeLen;
    }

    EEPROMindex += sizeof(AnalogueConfigData.dataBytes);

#ifdef DEBUG
    Serial.print("Analogue Checksum written: ");
    Serial.print(checksum, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif

    // Write CRC
    EEPROMext.EepromWrite(EEPROMindex, sizeof(int32Buf), int32Buf);
    EEPROMext.EepromWaitEndWriteOperation();

    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();
}

bool LoadAnalogueConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);
    // Set valid CRC flag to false
    bool validCRC = false;

    // Reset EEPROM index, analogue config comes straight after storage info
    EEPROMindex = sizeof(ChannelConfigData.dataBytes) + sizeof(uint32_t) + sizeof(SystemConfigData.dataBytes) + sizeof(uint32_t) +
                  sizeof(StorageConfigData.dataBytes) + sizeof(uint32_t);

    // Reset CRC result
    uint32_t result = 0;

    uint8_t int32Buf[4];
    // Read analogue config data in 32-byte chunks (page-safe)
    const uint16_t totalSize = sizeof(AnalogueConfigData.dataBytes);
    uint16_t bytesRemaining = totalSize;
    uint16_t addr = EEPROMindex;
    uint8_t *dst = AnalogueConfigData.dataBytes;

    while (bytesRemaining > 0)
    {
        uint8_t chunk = (bytesRemaining > 32) ? 32 : bytesRemaining;

        EEPROMext.EepromRead(addr, chunk, dst);

        addr += chunk;
        dst += chunk;
        bytesRemaining -= chunk;
    }

    EEPROMindex += totalSize;

    // Read stored CRC
    EEPROMext.EepromRead(EEPROMindex, sizeof(int32Buf), int32Buf);

    result = (uint32_t(int32Buf[0]) << 24) |
             (uint32_t(int32Buf[1]) << 16) |
             (uint32_t(int32Buf[2]) << 8) |
             (uint32_t(int32Buf[3]));
#ifdef DEBUG
    Serial.print("Analogue Checksum read: ");
    Serial.print(result, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif
    // Calculate read config bytes CRC
    uint32_t checksum = CRC32::calculate(AnalogueConfigData.dataBytes, sizeof(AnalogueConfigData.dataBytes));

    // Check stored CRC vs calculated CRC
    if (result == checksum)
    {
        validCRC = true;
        // Copy analogue input info
        memcpy(&AnalogueIns, &AnalogueConfigData.data, sizeof(AnalogueIns));
    }

    // Reset EEPROM index
    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();

    return validCRC;
}

void CleanEEPROM()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);

    uint8_t dummy[32];
    memset(dummy, 0xFF, sizeof(dummy));

    for (uint16_t addr = 0; addr < 8192; addr += 32)
    {
        EEPROMext.EepromWrite(addr, 32, dummy);
        EEPROMext.EepromWaitEndWriteOperation();
    }

    EEPROMext.end();
    SPI_2.end();
}

void InitialiseStorageData()
{
    if (StorageParams.LogFrequency == 0)
    {
        StorageParams.LogFrequency = DEFAULT_LOG_FREQUENCY;
    }
    if (StorageParams.MaxLogLength == 0)
    {
        StorageParams.MaxLogLength = DEFAULT_LOG_LINES;
    }
}

void InitialiseSD()
{
    // Attempt to begin SD if this is the first init after boot or there was a problem
    if (!SDCardOK)
    {
        SD.setDx(PC8, PC9, PC10, PC11);
        SD.setCMD(PD2);
        SD.setCK(PC12);
        SDCardOK = SD.begin();
        HAL_NVIC_DisableIRQ(SDIO_IRQn);
        HAL_NVIC_ClearPendingIRQ(SDIO_IRQn);
        HAL_NVIC_EnableIRQ(SDIO_IRQn);
        HAL_NVIC_SetPriority(SDIO_IRQn, 0, 0);
        if (!SDCardOK)
        {
#ifdef DEBUG
            Serial.println("SD Begin error");
#endif
        }
#ifdef DEBUG
        Serial.print("SD Card begin OK: ");
        Serial.println(SDCardOK);
#endif
    }

    // Card present, continue
    if (SDCardOK)
    {
        // Filename format is: YYYY-MM-DD_HH-MM-SS.csv
        sprintf(fileName, "%04d-%02d-%02d_%02d-%02d-%02d.csv", (2000 + rtc.getYear()), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

        // Create new file
        dataFile = SD.open(fileName, FILE_WRITE);

        // Create file was succesful. Write the header.
        if (dataFile)
        {
            // Deal with log rotation
            if (!logs.isFull())
            {
                // Circular buffer isn't full yet. Just unshit another file name in
                logs.unshift(fileName);
            }
            else
            {
                // Buffer is full. Delete the oldest file first before unshifting the new one
                char fileToDelete[24];
                logs.last().toCharArray(fileToDelete, sizeof(fileToDelete));
                if (SD.exists(fileToDelete))
                {
                    SD.remove(fileToDelete);
                }
                logs.unshift(fileName);
            }

            // Clear all log file names first
            memset(StorageParams.LogFileNames, 0, sizeof(StorageParams.LogFileNames));

            // Copy the circular buffer to storage parameters and save to EEPROM
            for (int i = 0; i < logs.size(); i++)
            {
                logs[i].toCharArray(StorageParams.LogFileNames[i], sizeof(StorageParams.LogFileNames[i]));
            }

            SaveStorageConfig();

            BytesStored = 0;
            SDFileOpen = true;

            // Print the file header to the buffer
            BytesStored += dataFile.print(systemHeader);

            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                BytesStored += dataFile.print(channelHeader);

                // Print a separating comma unless we're on the last channel
                if (i < NUM_CHANNELS - 1)
                {
                    BytesStored += dataFile.print(",");
                }
            }

            BytesStored += dataFile.println();
        }
        else
        {
            SDCardOK = false;
            SDFileOpen = false;
        }

#ifdef DEBUG
        Serial.println("SD Card init complete.");
#endif

        // Clear the undervoltage latch flag
        UndervoltageLatch = false;

        // Reset the line counter
        lineCount = 0;
    }
}

extern SD_HandleTypeDef uSdHandle;
void LogData()
{
    char timeStamp[100];
    char sysLog[150];
    char channelLog[150];
    int writtenBytes = 0;
    if (!(SystemRuntimeParams.ErrorFlags & UNDERVOLTAGE) && SDCardOK)
    {
        // Timestamp
        snprintf(timeStamp, sizeof(timeStamp), "%04d-%02d-%02d,%02d:%02d:%02d.%04d,", (2000 + rtc.getYear()), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getSubSeconds() % 1000);
        writtenBytes = dataFile.write(timeStamp, strlen(timeStamp));
        if (writtenBytes == 0)
        {
            // Clear flags for next attempt
            __HAL_SD_CLEAR_FLAG(&uSdHandle, SDIO_STATIC_FLAGS);
            SDCardOK = false;
            CloseSDFile();
            InitialiseSD();
            return;
        }
        BytesStored += writtenBytes;

        // System Parameters Log
        snprintf(sysLog, sizeof(sysLog), "%d,%.2f,%.2f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.2f,%.2f,%.2f,", SystemRuntimeParams.SystemTemperature, SystemRuntimeParams.VBatt, SystemRuntimeParams.SystemCurrent,
                 SystemRuntimeParams.ErrorFlags, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, lat, lon, alt, speed, accuracy);
        writtenBytes = dataFile.write(sysLog, strlen(sysLog));
        if (writtenBytes == 0)
        {
            // Clear flags for next attempt
            __HAL_SD_CLEAR_FLAG(&uSdHandle, SDIO_STATIC_FLAGS);
            SDCardOK = false;
            CloseSDFile();
            InitialiseSD();
            return;
        }
        BytesStored += writtenBytes;

        // Channel Data Logging
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            char chanType[6];
            char logEntry[40];
            switch (Channels[i].ChanType)
            {
            case DIG:
                snprintf(chanType, sizeof(chanType), "%s", "DIG");
                break;
            case DIG_PWM:
                snprintf(chanType, sizeof(chanType), "%s", "PWM");
                break;
            case ANA:
                snprintf(chanType, sizeof(chanType), "%s", "ANA");
                break;
            case ANA_PWM:
                snprintf(chanType, sizeof(chanType), "%s", "ANAP");
                break;
            case CAN_DIGITAL:
                snprintf(chanType, sizeof(chanType), "%s", "CAN");
                break;
            case CAN_PWM:
                snprintf(chanType, sizeof(chanType), "%s", "CANP");
                break;
            }
            snprintf(channelLog, sizeof(channelLog), "%s,%d,%.2f,%.2f,%.2f,%d,%d,%d%s",
                     chanType,
                     Channels[i].Enabled,
                     ChannelRuntime[i].CurrentValue,
                     Channels[i].CurrentThresholdHigh,
                     Channels[i].CurrentThresholdLow,
                     Channels[i].MultiChannel,
                     Channels[i].GroupNumber,
                     ChannelRuntime[i].ErrorFlags,
                     (i < NUM_CHANNELS - 1) ? "," : "\n");
            writtenBytes = dataFile.write(channelLog, strlen(channelLog));
            BytesStored += writtenBytes;
            if (writtenBytes == 0)
            {
                // Clear flags for next attempt
                __HAL_SD_CLEAR_FLAG(&uSdHandle, SDIO_STATIC_FLAGS);
                SDCardOK = false;
                CloseSDFile();
                InitialiseSD();
                return;
            }
        }

        // Periodic SD Flushing
        lineCount++;
        dataFile.flush();
        if (lineCount == StorageParams.MaxLogLength)
        {
            dataFile.close();
            SDFileOpen = false;
            InitialiseSD();
        }
    }
    else
    {
        // Handle SD or undervoltage errors
        if (!UndervoltageLatch)
        {
            CloseSDFile();
            UndervoltageLatch = true;
        }
        if (SystemRuntimeParams.ErrorFlags & UNDERVOLTAGE)
        {
            SDCardOK = false;
        }
        else
        {
            InitialiseSD();
        }
    }
}

void ResumeSD()
{
    // Attempt to begin SD if needed
    if (!SDCardOK)
    {
        SD.setDx(PC8, PC9, PC10, PC11);
        SD.setCMD(PD2);
        SD.setCK(PC12);
        SDCardOK = SD.begin();
        HAL_NVIC_DisableIRQ(SDIO_IRQn);
        HAL_NVIC_ClearPendingIRQ(SDIO_IRQn);
        HAL_NVIC_EnableIRQ(SDIO_IRQn);
        HAL_NVIC_SetPriority(SDIO_IRQn, 0, 0);
        if (!SDCardOK)
        {
#ifdef DEBUG
            Serial.println("SD Begin error during resume");
#endif
            return;
        }
    }

    if (SDCardOK)
    {
        // Get the most recent log file from the circular buffer
        if (logs.size() > 0)
        {
            char lastFileName[24] = {0};
            logs[0].toCharArray(lastFileName, sizeof(lastFileName));

            // Open the existing file in append mode
            dataFile = SD.open(lastFileName, FILE_WRITE);

            if (dataFile)
            {
                // Seek to end of file for appending
                dataFile.seek(dataFile.size());
                SDFileOpen = true;

#ifdef DEBUG
                Serial.print("Resumed logging to file: ");
                Serial.println(lastFileName);
#endif
            }
            else
            {
                SDCardOK = false;
                SDFileOpen = false;
#ifdef DEBUG
                Serial.print("Failed to open file for resume: ");
                Serial.println(lastFileName);
#endif
            }
        }
        else
        {
            // No log files in buffer, create a new one
            InitialiseSD();
        }

        // Clear the undervoltage latch flag
        UndervoltageLatch = false;
    }
}

void CloseSDFile()
{
    if (SDFileOpen)
    {
        dataFile.flush();
        dataFile.close();
        SDFileOpen = false;
    }
    SD.end();
}

void CleanupOrphanedLogFiles()
{
    if (!SDCardOK)
    {
#ifdef DEBUG
        Serial.println("SD card not ready for cleanup");
#endif
        return;
    }

    File root = SD.open("/");
    if (!root)
    {
#ifdef DEBUG
        Serial.println("Failed to open root directory");
#endif
        return;
    }

    File entry;
    int deletedCount = 0;

    while (entry = root.openNextFile())
    {
        // Only process files, not directories
        if (!entry.isDirectory())
        {
            char fileName[24] = {0};
            strncpy(fileName, entry.name(), sizeof(fileName) - 1);

            // Check if this file exists in the logs buffer
            bool fileInLogs = false;
            for (int i = 0; i < logs.size(); i++)
            {
                if (logs[i] == String(fileName))
                {
                    fileInLogs = true;
                    break;
                }
            }

            // If file is not in logs, delete it
            if (!fileInLogs)
            {
                entry.close();
                SD.remove(fileName);
                deletedCount++;
#ifdef DEBUG
                Serial.print("Deleted orphaned log file: ");
                Serial.println(fileName);
#endif
            }
        }
        entry.close();
    }
    root.close();

#ifdef DEBUG
    Serial.print("Cleanup complete. Files deleted: ");
    Serial.println(deletedCount);
#endif
}

void SleepSD()
{
    CloseSDFile();
    EEPROMext.end();
    SPI_2.end();

    // Disable SPI2 RCC clock to reduce power consumption during sleep
    __HAL_RCC_SPI2_CLK_DISABLE();
}
