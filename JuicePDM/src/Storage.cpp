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

STM32RTC &rtc2 = STM32RTC::getInstance();

M95640R EEPROMext(&SPI_2, CS1);

const char systemHeader[] = "Date,Time,System Temp,System Voltage,System Current,Error Flags,IMU Accel X,IMU Accel Y,IMU Accel Z,IMU Gyro X,IMU Gyro Y,IMU Gyro Z,Lat,Lon,Alt,Speed,Accuracy,";
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
        SDCardOK = SD.begin();
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
        sprintf(fileName, "%04d-%02d-%02d_%02d-%02d-%02d.csv", 2000 + rtc2.getYear(), rtc2.getMonth(), rtc2.getDay(), rtc2.getHours(), rtc2.getMinutes(), rtc2.getSeconds());

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

#define BUFFER_SIZE 4096 // Adjust buffer size as needed
char logBuffer[BUFFER_SIZE];
uint16_t bufferIndex = 0;

void FlushBuffer()
{
    // TODO: write to data file periodically returns zero and gets stuck. Find the root cause.
    int bytesNow = BytesStored;
    if (bufferIndex > 0)
    {
        BytesStored += dataFile.write(logBuffer, bufferIndex);
        dataFile.flush();
        bufferIndex = 0;

        if (BytesStored - bytesNow == 0)
        {
            Serial.println("SD write error");
            dataFile.close();
            InitialiseSD();
        }
    }
}

void AppendToBuffer(const char *data)
{
    size_t len = strlen(data);
    if (bufferIndex + len >= BUFFER_SIZE)
    {
        FlushBuffer();
    }
    strncpy(logBuffer + bufferIndex, data, len);
    bufferIndex += len;
}

void LogData()
{
    if (!(SystemParams.ErrorFlags & UNDERVOLTAGE) && SDCardOK)
    {
        // Timestamp
        char dateTimeStamp[32];
        snprintf(dateTimeStamp, sizeof(dateTimeStamp), "%04d-%02d-%02d,%02d:%02d:%02d,",
                 2000 + rtc2.getYear(), rtc2.getMonth(), rtc2.getDay(),
                 rtc2.getHours(), rtc2.getMinutes(), rtc2.getSeconds());
        AppendToBuffer(dateTimeStamp);

        // Buffers for float conversion
        char vbattBuffer[16], sysCurrentBuffer[16], accelXBuffer[16], accelYBuffer[16], accelZBuffer[16];
        char gyroXBuffer[16], gyroYBuffer[16], gyroZBuffer[16], latBuffer[16], lonBuffer[16];
        char altBuffer[16], speedBuffer[16], accuracyBuffer[16];

        // Convert floating-point values using dtostrf
        dtostrf(SystemParams.VBatt, 6, 2, vbattBuffer);
        dtostrf(SystemParams.SystemCurrent, 6, 2, sysCurrentBuffer);
        dtostrf(accelX, 6, 2, accelXBuffer);
        dtostrf(accelY, 6, 2, accelYBuffer);
        dtostrf(accelZ, 6, 2, accelZBuffer);
        dtostrf(gyroX, 6, 2, gyroXBuffer);
        dtostrf(gyroY, 6, 2, gyroYBuffer);
        dtostrf(gyroZ, 6, 2, gyroZBuffer);
        dtostrf(lat, 10, 6, latBuffer);
        dtostrf(lon, 10, 6, lonBuffer);
        dtostrf(alt, 6, 2, altBuffer);
        dtostrf(speed, 6, 2, speedBuffer);
        dtostrf(accuracy, 6, 2, accuracyBuffer);

        // System Parameters Log
        char logEntry[256];
        snprintf(logEntry, sizeof(logEntry),
                 "%d,%s,%s,%d,",
                 SystemParams.SystemTemperature,
                 vbattBuffer,
                 sysCurrentBuffer,
                 SystemParams.ErrorFlags);
        AppendToBuffer(logEntry);

        // Sensor Data
        snprintf(logEntry, sizeof(logEntry),
                 "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,",
                 accelXBuffer, accelYBuffer, accelZBuffer,
                 gyroXBuffer, gyroYBuffer, gyroZBuffer,
                 latBuffer, lonBuffer, altBuffer,
                 speedBuffer, accuracyBuffer);
        AppendToBuffer(logEntry);

        // Channel Data Logging
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            char valueBuffer[16], highThreshBuffer[16], lowThreshBuffer[16];
            dtostrf(Channels[i].CurrentValue, 6, 2, valueBuffer);
            dtostrf(Channels[i].CurrentThresholdHigh, 6, 2, highThreshBuffer);
            dtostrf(Channels[i].CurrentThresholdLow, 6, 2, lowThreshBuffer);

            snprintf(logEntry, sizeof(logEntry),
                     "%s,%d,%s,%s,%s,%d,%d,%d%s",
                     (Channels[i].ChanType == DIG) ? "DIG" : (Channels[i].ChanType == PWM)       ? "PWM"
                                                          : (Channels[i].ChanType == ANA)         ? "ANA"
                                                          : (Channels[i].ChanType == CAN_DIGITAL) ? "CAND"
                                                                                                  : "CANP",
                     Channels[i].Enabled,
                     valueBuffer, highThreshBuffer, lowThreshBuffer,
                     Channels[i].MultiChannel,
                     Channels[i].GroupNumber,
                     Channels[i].ErrorFlags,
                     (i < NUM_CHANNELS - 1) ? "," : "\n");
            AppendToBuffer(logEntry);
        }

        // Periodic SD Flushing
        lineCount++;
        if (lineCount == StorageParams.MaxLogLength)
        {
            FlushBuffer();
            dataFile.flush();
            dataFile.close();
            InitialiseSD();
        }
    }
    else
    {
        // Handle SD or undervoltage errors
        if (!UndervoltageLatch)
        {
            FlushBuffer();
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
