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
#include "FirmwareUpdate.h"
#include "InputHandler.h"
#include <stdarg.h>

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

static File transferFile;
static bool transferFileOpen = false;
static uint32_t transferBytesRead = 0;
static uint32_t transferTotalBytes = 0;
static bool transferSuspendedLogging = false;
static uint32_t lastLogFlushMillis = 0;
static size_t pendingLogBytes = 0;
static char pendingLogBuffer[8192] = {0};

static void RestoreLoggingAfterFailedTransferStart()
{
    if (transferSuspendedLogging)
    {
        transferSuspendedLogging = false;
        ResumeSD();
    }
}

uint32_t lineCount;

M95640R EEPROMext(&SPI_2, CS1);

const char systemHeader[] = "Date,Time,System Temp,SIM Module Temp,IMU Temp,System Voltage,System Current,Error Flags,IMU Accel X,IMU Accel Y,IMU Accel Z,IMU Gyro X,IMU Gyro Y,IMU Gyro Z,IMU Mag X,IMU Mag Y,IMU Mag Z,Lat,Lon,Alt,Speed,Accuracy,";
const char channelHeader[] = "Channel Type,Enabled,Current Value,Current Threshold High,Current Threshold Low,Multi-Channel,Group Number,Channel Error Flags,Analogue Input";
static const uint32_t LOG_FLUSH_INTERVAL_MS = 1000;
static const size_t LOG_LINE_BUFFER_SIZE = 1024;

static bool AppendFormattedText(char *buffer, size_t bufferSize, size_t *writeIndex, const char *format, ...)
{
    if (buffer == nullptr || writeIndex == nullptr || *writeIndex >= bufferSize)
    {
        return false;
    }

    va_list args;
    va_start(args, format);
    int written = vsnprintf(&buffer[*writeIndex], bufferSize - *writeIndex, format, args);
    va_end(args);

    if (written < 0 || static_cast<size_t>(written) >= (bufferSize - *writeIndex))
    {
        return false;
    }

    *writeIndex += static_cast<size_t>(written);
    return true;
}

static bool AppendText(char *buffer, size_t bufferSize, size_t *writeIndex, const char *text)
{
    if (buffer == nullptr || writeIndex == nullptr || text == nullptr || *writeIndex >= bufferSize)
    {
        return false;
    }

    size_t textLength = strlen(text);
    if (textLength >= (bufferSize - *writeIndex))
    {
        return false;
    }

    memcpy(&buffer[*writeIndex], text, textLength);
    *writeIndex += textLength;
    buffer[*writeIndex] = '\0';
    return true;
}

static bool AppendChar(char *buffer, size_t bufferSize, size_t *writeIndex, char value)
{
    if (buffer == nullptr || writeIndex == nullptr || (*writeIndex + 1) >= bufferSize)
    {
        return false;
    }

    buffer[*writeIndex] = value;
    (*writeIndex)++;
    buffer[*writeIndex] = '\0';
    return true;
}

static bool InitialiseSDCardInterface()
{
    if (SDCardOK)
    {
        return true;
    }

    SD.setDx(PC8, PC9, PC10, PC11);
    SD.setCMD(PD2);
    SD.setCK(PC12);
    SDCardOK = SD.begin();
    HAL_NVIC_DisableIRQ(SDIO_IRQn);
    HAL_NVIC_ClearPendingIRQ(SDIO_IRQn);
    HAL_NVIC_EnableIRQ(SDIO_IRQn);
    HAL_NVIC_SetPriority(SDIO_IRQn, 0, 0);

    return SDCardOK;
}

static void PersistLogHistory()
{
    memset(StorageParams.LogFileNames, 0, sizeof(StorageParams.LogFileNames));

    for (int i = 0; i < logs.size(); i++)
    {
        logs[i].toCharArray(StorageParams.LogFileNames[i], sizeof(StorageParams.LogFileNames[i]));
    }

    SaveStorageConfig();
}

static bool WriteCurrentLogHeader()
{
    pendingLogBytes = 0;
    BytesStored += dataFile.print(systemHeader);

    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        BytesStored += dataFile.print(channelHeader);
        BytesStored += dataFile.print(",");
    }

    for (int i = 0; i < NUM_DI_CHANNELS; i++)
    {
        BytesStored += dataFile.printf("Digital Input %d,", i + 1);
    }

    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        BytesStored += dataFile.printf("Analogue Input %d%s", i + 1, (i < NUM_ANA_CHANNELS - 1) ? "," : "");
    }

    BytesStored += dataFile.println();
    dataFile.flush();
    lastLogFlushMillis = millis();

    return dataFile;
}

static bool OpenLogFile(const char *targetFileName, bool trackAsNewLog)
{
    if (targetFileName == nullptr || targetFileName[0] == '\0')
    {
        return false;
    }

    bool fileExists = SD.exists(targetFileName);
    dataFile = SD.open(targetFileName, FILE_WRITE);
    if (!dataFile)
    {
        SDFileOpen = false;
        return false;
    }

    dataFile.seek(dataFile.size());
    SDFileOpen = true;
    pendingLogBytes = 0;
    snprintf(fileName, sizeof(fileName), "%s", targetFileName);
    BytesStored = dataFile.size();
    lastLogFlushMillis = millis();

    if (trackAsNewLog)
    {
        if (!logs.isEmpty())
        {
            char latestFileName[24] = {0};
            logs[0].toCharArray(latestFileName, sizeof(latestFileName));
            if (strncmp(latestFileName, targetFileName, sizeof(latestFileName)) != 0)
            {
                if (logs.isFull())
                {
                    char fileToDelete[24] = {0};
                    logs.last().toCharArray(fileToDelete, sizeof(fileToDelete));
                    if (SD.exists(fileToDelete))
                    {
                        SD.remove(fileToDelete);
                    }
                }
                logs.unshift(targetFileName);
                PersistLogHistory();
            }
        }
        else
        {
            logs.unshift(targetFileName);
            PersistLogHistory();
        }
    }

    if (!fileExists || dataFile.size() == 0)
    {
        BytesStored = 0;
        lineCount = 0;
        return WriteCurrentLogHeader();
    }

    return true;
}

static bool RecoverSDLogging()
{
    CloseSDFile();
    SDCardOK = false;

    if (!InitialiseSDCardInterface())
    {
        return false;
    }

    if (fileName[0] != '\0' && OpenLogFile(fileName, false))
    {
        return true;
    }

    InitialiseSD();
    return SDFileOpen;
}

static bool FlushPendingLogBuffer(bool syncToCard)
{
    if (!SDFileOpen || pendingLogBytes == 0)
    {
        if (syncToCard && SDFileOpen)
        {
            dataFile.flush();
            lastLogFlushMillis = millis();
        }

        return true;
    }

    extern SD_HandleTypeDef uSdHandle;

    if (dataFile.write((const uint8_t *)pendingLogBuffer, pendingLogBytes) != pendingLogBytes)
    {
        __HAL_SD_CLEAR_FLAG(&uSdHandle, SDIO_STATIC_FLAGS);
        if (!RecoverSDLogging())
        {
            SDCardOK = false;
        }

        return false;
    }

    BytesStored += pendingLogBytes;
    pendingLogBytes = 0;

    if (syncToCard)
    {
        dataFile.flush();
        lastLogFlushMillis = millis();
    }

    return true;
}

static const char *GetAnalogueUnitSuffix(uint8_t units)
{
    switch (units)
    {
    case ANA_UNITS_VOLTS:
        return "V";
    case ANA_UNITS_AMPS:
        return "A";
    case ANA_UNITS_CELSIUS:
        return "C";
    case ANA_UNITS_FAHRENHEIT:
        return "F";
    case ANA_UNITS_PERCENT:
        return "%";
    case ANA_UNITS_RPM:
        return "RPM";
    case ANA_UNITS_KPH:
        return "kph";
    case ANA_UNITS_MPH:
        return "mph";
    case ANA_UNITS_BAR:
        return "bar";
    case ANA_UNITS_PSI:
        return "psi";
    default:
        return "";
    }
}

static int GetAnalogueInputIndexForPin(uint8_t inputPin)
{
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        if (ANAchannelInputPins[i] == inputPin)
        {
            return i;
        }
    }

    return -1;
}

static const char *GetChannelTypeLabel(ChannelType channelType)
{
    switch (channelType)
    {
    case DIG:
        return "DIG";
    case DIG_PWM:
        return "PWM";
    case ANA:
        return "ANA";
    case ANA_PWM:
        return "ANAP";
    case CAN_DIGITAL:
        return "CAN";
    case CAN_PWM:
        return "CANP";
    case DIG_INTERMITTENT:
        return "INT";
    default:
        return "UNK";
    }
}

static void FormatAnalogueInputLogValue(int analogueInputIndex, char *outputBuffer, size_t outputBufferSize)
{
    if (outputBuffer == nullptr || outputBufferSize == 0)
    {
        return;
    }

    outputBuffer[0] = '-';
    if (outputBufferSize > 1)
    {
        outputBuffer[1] = '\0';
    }

    if (analogueInputIndex < 0 || analogueInputIndex >= NUM_ANA_CHANNELS)
    {
        return;
    }

    float inputValue = AnalogueIns[analogueInputIndex].InputValue;
    const char *unitSuffix = GetAnalogueUnitSuffix(AnalogueIns[analogueInputIndex].Units);
    snprintf(outputBuffer, outputBufferSize, "%.1f%s", inputValue, unitSuffix);
}

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
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            SanitizeChannelConfig(Channels[i]);
        }
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

    // Write CRC (4 bytes) in page-safe chunks
    uint8_t crcBuf[4];
    crcBuf[0] = (checksum >> 24) & 0xFF;
    crcBuf[1] = (checksum >> 16) & 0xFF;
    crcBuf[2] = (checksum >> 8) & 0xFF;
    crcBuf[3] = checksum & 0xFF;

    uint16_t crcAddr = EEPROMindex;
    uint8_t *crcSrc = crcBuf;
    uint8_t crcBytesRemaining = sizeof(crcBuf);

    while (crcBytesRemaining > 0)
    {
        uint8_t pageOffset = crcAddr % EEPROM_PAGE_SIZE;
        uint8_t spaceInPage = EEPROM_PAGE_SIZE - pageOffset;
        uint8_t writeLen = (crcBytesRemaining < spaceInPage) ? crcBytesRemaining : spaceInPage;

        EEPROMext.EepromWrite(crcAddr, writeLen, crcSrc);
        EEPROMext.EepromWaitEndWriteOperation();

        crcAddr += writeLen;
        crcSrc += writeLen;
        crcBytesRemaining -= writeLen;
    }

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
        for (int i = 9; i >= 0; i--)
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

    // Write CRC in page-safe chunks. The analogue config grew enough that the
    // trailing CRC can now straddle a page boundary.
    uint16_t crcAddr = EEPROMindex;
    uint8_t *crcSrc = int32Buf;
    uint8_t crcBytesRemaining = sizeof(int32Buf);

    while (crcBytesRemaining > 0)
    {
        uint8_t pageOffset = crcAddr % EEPROM_PAGE_SIZE;
        uint8_t spaceInPage = EEPROM_PAGE_SIZE - pageOffset;
        uint8_t writeLen = (crcBytesRemaining < spaceInPage) ? crcBytesRemaining : spaceInPage;

        EEPROMext.EepromWrite(crcAddr, writeLen, crcSrc);
        EEPROMext.EepromWaitEndWriteOperation();

        crcAddr += writeLen;
        crcSrc += writeLen;
        crcBytesRemaining -= writeLen;
    }

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

        for (int i = 0; i < NUM_ANA_CHANNELS; i++)
        {
            if (SyncChannelTypesForAnalogueInput(i))
            {
                saveEEPROMOnTimeout = true;
                EEPROMSaveTimout = millis() + EEPROM_WRITE_DELAY;
            }
        }
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
    if (!InitialiseSDCardInterface())
    {
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

        if (!OpenLogFile(fileName, true))
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

void LogData()
{
    char logLine[LOG_LINE_BUFFER_SIZE] = {0};
    size_t logLineIndex = 0;
    size_t expectedBytes = 0;

    if (transferSuspendedLogging)
    {
        return;
    }

    if (IsFirmwareAssetUploadInProgress())
    {
        return;
    }

    if (!(SystemRuntimeParams.ErrorFlags & UNDERVOLTAGE) && SDCardOK)
    {
        if (!AppendFormattedText(logLine,
                                 sizeof(logLine),
                                 &logLineIndex,
                                 "%04d-%02d-%02d,%02d:%02d:%02d.%03lu,",
                                 (2000 + rtc.getYear()),
                                 rtc.getMonth(),
                                 rtc.getDay(),
                                 rtc.getHours(),
                                 rtc.getMinutes(),
                                 rtc.getSeconds(),
                                 static_cast<unsigned long>(rtc.getSubSeconds())))
        {
            return;
        }

        float loggedLat = GPSFix ? lat : 0.0f;
        float loggedLon = GPSFix ? lon : 0.0f;
        float loggedAlt = GPSFix ? alt : 0.0f;
        float loggedSpeed = GPSFix ? speed : 0.0f;
        float loggedAccuracy = GPSFix ? accuracy : 0.0f;

        if (!AppendFormattedText(logLine,
                                 sizeof(logLine),
                                 &logLineIndex,
                                 "%d,%.1f,%.1f,%.1f,%.1f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.2f,%.2f,%.2f,",
                                 SystemRuntimeParams.SystemTemperature,
                                 SystemRuntimeParams.SIMModuleTemp,
                                 SystemRuntimeParams.IMUTemp,
                                 SystemRuntimeParams.VBatt,
                                 SystemRuntimeParams.SystemCurrent,
                                 SystemRuntimeParams.ErrorFlags,
                                 accelX,
                                 accelY,
                                 accelZ,
                                 gyroX,
                                 gyroY,
                                 gyroZ,
                                 magX,
                                 magY,
                                 magZ,
                                 loggedLat,
                                 loggedLon,
                                 loggedAlt,
                                 loggedSpeed,
                                 loggedAccuracy))
        {
            return;
        }

        // Channel Data Logging
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            char analogueInputLog[32];
            const char *chanType = GetChannelTypeLabel(Channels[i].ChanType);
            analogueInputLog[0] = '-';
            analogueInputLog[1] = '\0';
            if (Channels[i].ChanType == ANA || Channels[i].ChanType == ANA_PWM)
            {
                int analogueInputIndex = GetAnalogueInputIndexForPin(Channels[i].InputControlPin);
                FormatAnalogueInputLogValue(analogueInputIndex, analogueInputLog, sizeof(analogueInputLog));
            }

            if (!AppendFormattedText(logLine,
                                     sizeof(logLine),
                                     &logLineIndex,
                                     "%s,%d,%.1f,%.1f,%.1f,%d,%d,%d,%s,",
                                     chanType,
                                     Channels[i].Enabled,
                                     ChannelRuntime[i].CurrentValue,
                                     Channels[i].CurrentThresholdHigh,
                                     Channels[i].CurrentThresholdLow,
                                     Channels[i].MultiChannel,
                                     Channels[i].GroupNumber,
                                     ChannelRuntime[i].ErrorFlags,
                                     analogueInputLog))
            {
                return;
            }
        }

        for (int i = 0; i < NUM_DI_CHANNELS; i++)
        {
            if (!AppendChar(logLine,
                            sizeof(logLine),
                            &logLineIndex,
                            digitalRead(DIchannelInputPins[i]) ? '1' : '0') ||
                !AppendChar(logLine,
                            sizeof(logLine),
                            &logLineIndex,
                            ','))
            {
                return;
            }
        }

        for (int i = 0; i < NUM_ANA_CHANNELS; i++)
        {
            char analogueInputLog[32];
            FormatAnalogueInputLogValue(i, analogueInputLog, sizeof(analogueInputLog));
            if (!AppendText(logLine,
                            sizeof(logLine),
                            &logLineIndex,
                            analogueInputLog) ||
                !AppendChar(logLine,
                            sizeof(logLine),
                            &logLineIndex,
                            (i < NUM_ANA_CHANNELS - 1) ? ',' : '\n'))
            {
                return;
            }
        }

        expectedBytes = logLineIndex;
        if ((pendingLogBytes + expectedBytes) > sizeof(pendingLogBuffer))
        {
            if (!FlushPendingLogBuffer(false))
            {
                return;
            }
        }

        memcpy(&pendingLogBuffer[pendingLogBytes], logLine, expectedBytes);
        pendingLogBytes += expectedBytes;

        // Periodic SD Flushing
        lineCount++;
        if ((millis() - lastLogFlushMillis) >= LOG_FLUSH_INTERVAL_MS)
        {
            if (!FlushPendingLogBuffer(true))
            {
                return;
            }
        }
        if (lineCount == StorageParams.MaxLogLength)
        {
            FlushPendingLogBuffer(true);
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
    if (!InitialiseSDCardInterface())
    {
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
        if (fileName[0] != '\0' && OpenLogFile(fileName, false))
        {
            UndervoltageLatch = false;
            return;
        }

        // Get the most recent log file from the circular buffer
        if (logs.size() > 0)
        {
            char lastFileName[24] = {0};
            logs[0].toCharArray(lastFileName, sizeof(lastFileName));

            if (OpenLogFile(lastFileName, false))
            {
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
        FlushPendingLogBuffer(true);
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
    CancelLogTransfer();
    EEPROMext.end();
    SPI_2.end();

    // Disable SPI2 RCC clock to reduce power consumption during sleep
    __HAL_RCC_SPI2_CLK_DISABLE();
}

uint8_t GetAvailableLogFileCount()
{
    return logs.size();
}

bool GetLogFileNameByIndex(uint8_t index, char *outFileName, size_t outSize)
{
    if (outFileName == nullptr || outSize == 0 || index >= logs.size())
    {
        return false;
    }

    memset(outFileName, 0, outSize);
    logs[index].toCharArray(outFileName, outSize);
    return true;
}

bool GetLogFileSizeByIndex(uint8_t index, uint32_t *outSizeBytes)
{
    if (outSizeBytes == nullptr)
    {
        return false;
    }

    *outSizeBytes = 0;

    char selectedFile[24] = {0};
    if (!GetLogFileNameByIndex(index, selectedFile, sizeof(selectedFile)))
    {
        return false;
    }

    if (!InitialiseSDCardInterface())
    {
        return false;
    }

    if (!SD.exists(selectedFile))
    {
        return false;
    }

    File file = SD.open(selectedFile, FILE_READ);
    if (!file)
    {
        return false;
    }

    *outSizeBytes = file.size();
    file.close();
    return true;
}

bool BeginLogTransfer(uint8_t index)
{
    CancelLogTransfer();

    // Avoid SD read/write contention: pause active logging while transfer runs.
    if (SDFileOpen)
    {
        FlushPendingLogBuffer(true);
        dataFile.close();
        SDFileOpen = false;
        transferSuspendedLogging = true;
    }

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
    }

    if (!SDCardOK)
    {
        RestoreLoggingAfterFailedTransferStart();
        return false;
    }

    char selectedFile[24] = {0};
    if (!GetLogFileNameByIndex(index, selectedFile, sizeof(selectedFile)))
    {
        RestoreLoggingAfterFailedTransferStart();
        return false;
    }

    if (!SD.exists(selectedFile))
    {
        RestoreLoggingAfterFailedTransferStart();
        return false;
    }

    transferFile = SD.open(selectedFile, FILE_READ);
    if (!transferFile)
    {
        RestoreLoggingAfterFailedTransferStart();
        return false;
    }

    transferTotalBytes = transferFile.size();
    transferBytesRead = 0;
    transferFileOpen = true;
    return true;
}

bool ReadLogTransferChunk(char *outBuffer,
                          size_t outBufferSize,
                          uint16_t maxLines,
                          uint16_t *outBytesWritten,
                          uint8_t *outProgress,
                          bool *outDone)
{
    if (!transferFileOpen || outBuffer == nullptr || outBufferSize < 2 ||
        outBytesWritten == nullptr || outProgress == nullptr || outDone == nullptr)
    {
        return false;
    }

    size_t writeIndex = 0;
    uint16_t linesSent = 0;
    bool enforceLineLimit = maxLines > 0;

    if (!enforceLineLimit)
    {
        // Fast path: use a single block read when line boundaries are not required.
        int bytesRead = transferFile.read((uint8_t *)outBuffer, outBufferSize - 1);
        if (bytesRead > 0)
        {
            writeIndex = (size_t)bytesRead;
        }
    }
    else
    {
        while (transferFile.available() && linesSent < maxLines && writeIndex < (outBufferSize - 1))
        {
            int byteRead = transferFile.read();
            if (byteRead < 0)
            {
                break;
            }

            outBuffer[writeIndex++] = (char)byteRead;
            if (byteRead == '\n')
            {
                linesSent++;
            }
        }
    }

    outBuffer[writeIndex] = '\0';
    *outBytesWritten = writeIndex;

    transferBytesRead = transferFile.position();
    if (transferTotalBytes > 0)
    {
        *outProgress = (uint8_t)((transferBytesRead * 100UL) / transferTotalBytes);
    }
    else
    {
        *outProgress = 100;
    }

    *outDone = !transferFile.available();
    if (*outDone)
    {
        CancelLogTransfer();
        *outProgress = 100;
    }

    return true;
}

bool IsLogTransferActive()
{
    return transferFileOpen;
}

void CancelLogTransfer()
{
    if (transferFileOpen)
    {
        transferFile.close();
    }

    transferFileOpen = false;
    transferBytesRead = 0;
    transferTotalBytes = 0;

    if (transferSuspendedLogging)
    {
        transferSuspendedLogging = false;
        ResumeSD();
    }
}

bool ResetAllLogs()
{
    CancelLogTransfer();
    CloseSDFile();

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
        return false;
    }

    File root = SD.open("/");
    if (!root)
    {
        return false;
    }

    File entry;
    while (entry = root.openNextFile())
    {
        if (!entry.isDirectory())
        {
            char fileName[32] = {0};
            strncpy(fileName, entry.name(), sizeof(fileName) - 1);
            entry.close();
            SD.remove(fileName);
            continue;
        }

        entry.close();
    }

    root.close();

    logs.clear();
    memset(StorageParams.LogFileNames, 0, sizeof(StorageParams.LogFileNames));
    SaveStorageConfig();

    BytesStored = 0;
    SDFileOpen = false;
    fileName[0] = '\0';
    lineCount = 0;
    lastLogFlushMillis = 0;

    if (HasUsableRtcTime() && !(SystemRuntimeParams.ErrorFlags & UNDERVOLTAGE))
    {
        InitialiseSD();
        return SDFileOpen;
    }

    return true;
}
