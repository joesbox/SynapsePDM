/*  ConfigStorage.cpp Functions and variables for EEPROM storage and SD data logging.
    Copyright (c) 2023 Joe Mann.  All right reserved.

    This work is licensed under the Creative Commons
    Attribution-NonCommercial-ShareAlike 4.0 International License.
    To view a copy of this license, visit
    https://creativecommons.org/licenses/by-nc-sa/4.0/ or send a
    letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

    You are free to:
    - Share: Copy and redistribute the material in any medium or format.
    - Adapt: Remix, transform, and build upon the material.

    Under the following terms:
    - Attribution: You must give appropriate credit, provide a link to the license,
      and indicate if changes were made. You may do so in any reasonable manner,
      but not in any way that suggests the licensor endorses you or your use.
    - NonCommercial: You may not use the material for commercial purposes.
    - ShareAlike: If you remix, transform, or build upon the material,
      you must distribute your contributions under the same license as the original.

    DISCLAIMER: This software is provided "as is," without warranty of any kind,
    express or implied, including but not limited to the warranties of
    merchantability, fitness for a particular purpose, and noninfringement.
    In no event shall the authors or copyright holders be liable for any claim,
    damages, or other liability, whether in an action of contract, tort, or otherwise,
    arising from, out of, or in connection with the software or the use or
    other dealings in the software.
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

CircularBuffer<String, 10> logs;

uint32_t lineCount;

M95640R EEPROMext(&SPI_2, CS1);

const char systemHeader[] = "Date,Time,Backup Battery SoC,System Temp,System Voltage,System Current,Error Flags,IMU Accel X,IMU Accel Y,IMU Accel Z,IMU Gyro X,IMU Gyro Y,IMU Gyro Z,Lat,Lon,Alt,Speed,Accuracy,";
const char channelHeader[] = "Channel Type,Enabled,Current Value,Current Threshold High,Current Threshold Low,Multi-Channel,Group Number,Channel Error Flags";

long startMillis;
long endMillis;

void SaveChannelConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);

    // Reset EEPROM index
    EEPROMindex = 0;

    // Copy current channel info to storage structure
    memcpy(&ChannelConfigData.data, &Channels, sizeof(Channels));

    // Calculate stored config bytes CRC
    uint32_t checksum = CRC32::calculate(ChannelConfigData.dataBytes, sizeof(Channels));

    // Store data and CRC value
    uint8_t int32Buf[4];
    int32Buf[0] = (checksum >> 24) & 0xFF;
    int32Buf[1] = (checksum >> 16) & 0xFF;
    int32Buf[2] = (checksum >> 8) & 0xFF;
    int32Buf[3] = checksum & 0xFF;

    for (unsigned int i = 0; i < sizeof(ChannelConfigData.dataBytes); i++)
    {
        EEPROMext.EepromWrite(EEPROMindex, 1, &ChannelConfigData.dataBytes[i]);
        EEPROMindex++;
    }

#ifdef DEBUG
    Serial.print("Channel Checksum written: ");
    Serial.print(checksum, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif
    EEPROMext.EepromWrite(EEPROMindex, sizeof(checksum), int32Buf);
    EEPROMext.EepromWaitEndWriteOperation();

    // Reset EEPROM index
    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();
}

bool LoadChannelConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);
    // Set valid CRC flag to false
    bool validCRC = false;

    // Reset index
    EEPROMindex = 0;

    // Reset CRC result
    uint32_t result = 0;

    uint8_t int32Buf[4];

    for (unsigned int i = 0; i < sizeof(ChannelConfigData.dataBytes); i++)
    {
        EEPROMext.EepromRead(EEPROMindex, 1, &ChannelConfigData.dataBytes[i]);
        EEPROMindex++;
    }
    EEPROMext.EepromRead(EEPROMindex, sizeof(result), int32Buf);

    result = (int32_t(int32Buf[0]) << 24) |
             (int32_t(int32Buf[1]) << 16) |
             (int32_t(int32Buf[2]) << 8) |
             int32_t(int32Buf[3]);

#ifdef DEBUG
    Serial.print("Channel Checksum read: ");
    Serial.print(result, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif
    // Calculate read config bytes CRC
    uint32_t checksum = CRC32::calculate(ChannelConfigData.dataBytes, sizeof(Channels));

    // Check stored CRC vs calculated CRC
    if (result == checksum)
    {
        validCRC = true;
        // Copy channel info
        memcpy(&Channels, &ChannelConfigData.data, sizeof(Channels));
    }

    // Reset EEPROM index
    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();

    return validCRC;
}

void SaveSystemConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);

    // Reset EEPROM index, system info comes straight after channel info
    EEPROMindex = sizeof(ChannelConfigData.data) + sizeof(uint32_t);

    // Copy current system info to storage structure
    memcpy(&SystemConfigData.data, &SystemParams, sizeof(SystemParams));

    // Calculate stored config bytes CRC
    uint32_t checksum = CRC32::calculate(SystemConfigData.dataBytes, sizeof(SystemParams));

    // Store data and CRC value
    uint8_t int32Buf[4];
    int32Buf[0] = (checksum >> 24) & 0xFF;
    int32Buf[1] = (checksum >> 16) & 0xFF;
    int32Buf[2] = (checksum >> 8) & 0xFF;
    int32Buf[3] = checksum & 0xFF;

    for (unsigned int i = 0; i < sizeof(SystemConfigData.dataBytes); i++)
    {
        EEPROMext.EepromWrite(EEPROMindex, 1, &SystemConfigData.dataBytes[i]);
        EEPROMindex++;
    }

#ifdef DEBUG
    Serial.print("System Checksum written: ");
    Serial.print(checksum, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif
    EEPROMext.EepromWrite(EEPROMindex, sizeof(checksum), int32Buf);
    EEPROMext.EepromWaitEndWriteOperation();

    // Reset EEPROM index
    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();
}

bool LoadSystemConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);
    // Set valid CRC flag to false
    bool validCRC = false;

    // Reset EEPROM index, system info comes straight after channel info
    EEPROMindex = sizeof(ChannelConfigData.data) + sizeof(uint32_t);

    // Reset CRC result
    uint32_t result = 0;

    uint8_t int32Buf[4];

    for (unsigned int i = 0; i < sizeof(SystemConfigData.dataBytes); i++)
    {
        EEPROMext.EepromRead(EEPROMindex, 1, &SystemConfigData.dataBytes[i]);
        EEPROMindex++;
    }
    EEPROMext.EepromRead(EEPROMindex, sizeof(result), int32Buf);

    result = (int32_t(int32Buf[0]) << 24) |
             (int32_t(int32Buf[1]) << 16) |
             (int32_t(int32Buf[2]) << 8) |
             int32_t(int32Buf[3]);

#ifdef DEBUG
    Serial.print("System checksum read: ");
    Serial.print(result, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif
    // Calculate read config bytes CRC
    uint32_t checksum = CRC32::calculate(SystemConfigData.dataBytes, sizeof(SystemParams));

    // Check stored CRC vs calculated CRC
    if (result == checksum)
    {
        validCRC = true;
        // Copy system info
        memcpy(&SystemParams, &SystemConfigData.data, sizeof(SystemParams));
    }

    // Reset EEPROM index
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

    // Reset EEPROM index, storage info comes straight after system info
    EEPROMindex = sizeof(ChannelConfigData.data) + sizeof(uint32_t) + sizeof(SystemConfigData.data) + sizeof(uint32_t);

    // Copy current storage info to storage structure
    memcpy(&StorageConfigData.data, &StorageParams, sizeof(StorageParameters));

    // Calculate stored config bytes CRC
    uint32_t checksum = CRC32::calculate(StorageConfigData.dataBytes, sizeof(StorageParameters));

    // Store data and CRC value
    uint8_t int32Buf[4];
    int32Buf[0] = (checksum >> 24) & 0xFF;
    int32Buf[1] = (checksum >> 16) & 0xFF;
    int32Buf[2] = (checksum >> 8) & 0xFF;
    int32Buf[3] = checksum & 0xFF;

    for (unsigned int i = 0; i < sizeof(StorageConfigData.dataBytes); i++)
    {
        EEPROMext.EepromWrite(EEPROMindex, 1, &StorageConfigData.dataBytes[i]);
        EEPROMindex++;
    }

#ifdef DEBUG
    Serial.print("Storage Checksum written: ");
    Serial.print(checksum, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif
    EEPROMext.EepromWrite(EEPROMindex, sizeof(checksum), int32Buf);
    EEPROMext.EepromWaitEndWriteOperation();

    // Reset EEPROM index
    EEPROMindex = 0;
    EEPROMext.end();
    SPI_2.end();
}

bool LoadStorageConfig()
{
    SPI_2.begin();
    EEPROMext.begin(EEPROM_SPI_SPEED);
    // Set valid CRC flag to false
    bool validCRC = false;

    // Reset EEPROM index, storage info comes straight after channel info
    EEPROMindex = sizeof(ChannelConfigData.data) + sizeof(uint32_t) + sizeof(SystemConfigData.data) + sizeof(uint32_t);

    // Reset CRC result
    uint32_t result = 0;

    uint8_t int32Buf[4];

    for (unsigned int i = 0; i < sizeof(StorageConfigData.dataBytes); i++)
    {
        EEPROMext.EepromRead(EEPROMindex, 1, &StorageConfigData.dataBytes[i]);
        EEPROMindex++;
    }
    EEPROMext.EepromRead(EEPROMindex, sizeof(result), int32Buf);

    result = (int32_t(int32Buf[0]) << 24) |
             (int32_t(int32Buf[1]) << 16) |
             (int32_t(int32Buf[2]) << 8) |
             int32_t(int32Buf[3]);

#ifdef DEBUG
    Serial.print("Storage checksum read: ");
    Serial.print(result, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif
    // Calculate read config bytes CRC
    uint32_t checksum = CRC32::calculate(StorageConfigData.dataBytes, sizeof(StorageParams));

    // Check stored CRC vs calculated CRC
    if (result == checksum)
    {
        validCRC = true;
        // Copy storage info
        memcpy(&StorageParams, &StorageConfigData.data, sizeof(StorageParams));

        // Load the circular buffer with the log files we're currently keeping
        for (int i = 0; i < 10; i++)
        {
            if (strlen(StorageParams.LogFileNames[i]) != 0)
            {
                logs.unshift(StorageParams.LogFileNames[i]);
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
    uint8_t dummy[32] = {0xFF};
    for (int i = 0; i < 256; i++)
    {
        EEPROMext.EepromWrite(i, 32, dummy);
    }
    EEPROMext.EepromWaitEndWriteOperation();
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
        sprintf(fileName, "%04d-%02d-%02d_%02d-%02d-%02d.csv", (2000 + RTCyear), RTCmonth, RTCday, RTChour, RTCminute, RTCsecond);

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

            // Copy the circular buffer to storage parameters and save to EEPROM
            for (int i = 0; i < logs.size(); i++)
            {
                logs[i].toCharArray(StorageParams.LogFileNames[i], sizeof(StorageParams.LogFileNames[i]));
            }

            SaveStorageConfig();

            BytesStored = 0;

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
    if (!(SystemParams.ErrorFlags & UNDERVOLTAGE) && SDCardOK)
    {
        // Timestamp
        snprintf(timeStamp, sizeof(timeStamp), "%04d-%02d-%02d,%02d:%02d:%02d.%04d,", (2000 + RTCyear), RTCmonth, RTCday, RTChour, RTCminute, RTCsecond, millis() % 1000);
        writtenBytes = dataFile.write(timeStamp, strlen(timeStamp));
        if (writtenBytes == 0)
        {
            Serial.println("Logging failed on timestamp entry");
            Serial.println(HAL_SD_GetError(&uSdHandle));
            // Clear flags for next attempt
            __HAL_SD_CLEAR_FLAG(&uSdHandle, SDIO_STATIC_FLAGS);
            SDCardOK = false;
            CloseSDFile();
            InitialiseSD();
            return;
        }
        BytesStored += writtenBytes;

        // System Parameters Log
        snprintf(sysLog, sizeof(sysLog), "%d,%d,%.2f,%.2f,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.2f,%.2f,%.2f,", SOC, SystemParams.SystemTemperature, SystemParams.VBatt, SystemParams.SystemCurrent,
                 SystemParams.ErrorFlags, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, lat, lon, alt, speed, accuracy);
        writtenBytes = dataFile.write(sysLog, strlen(sysLog));
        if (writtenBytes == 0)
        {
            Serial.println("Logging failed on syslog entry");
            Serial.println(dataFile.getErrorstate());
            Serial.println(dataFile.getWriteError());
            Serial.println(HAL_SD_GetError(&uSdHandle));
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
                     Channels[i].CurrentValue,
                     Channels[i].CurrentThresholdHigh,
                     Channels[i].CurrentThresholdLow,
                     Channels[i].MultiChannel,
                     Channels[i].GroupNumber,
                     Channels[i].ErrorFlags,
                     (i < NUM_CHANNELS - 1) ? "," : "\n");
            writtenBytes = dataFile.write(channelLog, strlen(channelLog));
            BytesStored += writtenBytes;
            if (writtenBytes == 0)
            {
                Serial.println("Logging failed on channel entry");
                Serial.println(dataFile.getErrorstate());
                Serial.println(dataFile.getWriteError());
                Serial.println(HAL_SD_GetError(&uSdHandle));
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
        if (SystemParams.ErrorFlags & UNDERVOLTAGE)
        {
            SDCardOK = false;
        }
        else
        {
            InitialiseSD();
        }
    }
}

void CloseSDFile()
{
    dataFile.flush();
    dataFile.close();
    SD.end();
}

void SleepSD()
{
    CloseSDFile();
}
