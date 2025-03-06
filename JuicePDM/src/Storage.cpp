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
File myfile;
char fileName[23];
char dateTimeStamp[23];
int BytesStored;
bool UndervoltageLatch;
bool StorageCRCValid;

CircularBuffer<String, 10> logs;

uint32_t lineCount;

STM32RTC &rtc2 = STM32RTC::getInstance();

M95640R EEPROMext(&SPI_2, CS1);

void SaveChannelConfig()
{
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
}

bool LoadChannelConfig()
{
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

    return validCRC;
}

void SaveSystemConfig()
{
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
}

bool LoadSystemConfig()
{
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

    return validCRC;
}

void SaveStorageConfig()
{
    EEPROMext.begin(EEPROM_SPI_SPEED);

    // Reset EEPROM index, storage info comes straight after system info
    EEPROMindex = sizeof(ChannelConfigData.data) + sizeof(uint32_t) + sizeof(SystemConfigData.data) + sizeof(uint32_t);

    // Copy current storage info to storage structure
    memcpy(&StorageConfigData.data, &StorageParams, sizeof(StorageParams));

    // Calculate stored config bytes CRC
    uint32_t checksum = CRC32::calculate(StorageConfigData.dataBytes, sizeof(StorageParams));

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
}

bool LoadStorageConfig()
{
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
    Serial.print("System checksum read: ");
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
    }

    // Reset EEPROM index
    EEPROMindex = 0;
    EEPROMext.end();

    return validCRC;
}

void InitialiseStorageData()
{
    StorageParams.LogFrequency = DEFAULT_LOG_FREQUENCY;
    StorageParams.MaxLogLength = DEFAULT_LOG_LINES;
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
        myfile = SD.open(fileName, FILE_WRITE);
        if (!SDCardOK)
        {
#ifdef DEBUG
            Serial.println("SD init open error");
#endif
        }

        // Create file was succesful. Write the header.
        if (myfile)
        {
            // Deal with log rotation
            if (!logs.isFull())
            {
                // Circular buffer isn't full yet. Just unshit another file name in
                Serial.println(fileName);
                logs.unshift(fileName);
            }
            else
            {
                // Buffer is full. Delete the oldest file first before unshifting the new one
                char fileToDelete[24];
                logs.last().toCharArray(fileToDelete, sizeof(fileToDelete));
                Serial.println(fileToDelete);
                if (SD.exists(fileToDelete))
                {
                    SD.remove(fileToDelete);
                    logs.unshift(fileName);
                }
            }
            // Print the file header to the buffer
            myfile.print("Date,Time,System Temp,System Voltage,System Current,Error Flags,IMU Accel X,IMU Accel Y,IMU Accel Z,IMU Gyro X,IMU Gyro Y,IMU Gyro Z,Lat,Lon,Alt,Speed,Accuracy,");

            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                myfile.print("Channel Type,Enabled,Current Value,Current Threshold High,Current Threshold Low,PWM,Multi-Channel,Group Number,Channel Error Flags");

                // Print a separating comma unless we're on the last channel
                if (i < NUM_CHANNELS - 1)
                {
                    myfile.print(",");
                }
            }

            myfile.println();
        }
        else
        {
            SDCardOK = false;
        }

        BytesStored = 0;
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
    // Log if we're not in an undervoltage condition and the SD card is OK
    if (!(SystemParams.ErrorFlags & UNDERVOLTGAGE) && SDCardOK)
    {
        // Print date and time
        sprintf(dateTimeStamp, "%04d-%02d-%02d,%02d:%02d:%02d,", 2000 + rtc2.getYear(), rtc2.getMonth(), rtc2.getDay(), rtc2.getHours(), rtc2.getMinutes(), rtc2.getSeconds());
        myfile.print(dateTimeStamp);

        // Add system parameters
        myfile.print(SystemParams.SystemTemperature);
        myfile.print(",");

        myfile.print(SystemParams.VBatt);
        myfile.print(",");

        myfile.print(SystemParams.SystemCurrent);
        myfile.print(",");

        myfile.print(SystemParams.ErrorFlags, HEX);
        myfile.print(",");

        myfile.print(accelX);
        myfile.print(",");

        myfile.print(accelY);
        myfile.print(",");

        myfile.print(accelZ);
        myfile.print(",");

        myfile.print(gyroX);
        myfile.print(",");

        myfile.print(gyroY);
        myfile.print(",");

        myfile.print(gyroZ);
        myfile.print(",");

        myfile.print(lat, 6);
        myfile.print(",");

        myfile.print(lon, 6);
        myfile.print(",");

        myfile.print(alt);
        myfile.print(",");

        myfile.print(speed);
        myfile.print(",");

        myfile.print(accuracy);
        myfile.print(",");

        // Add info for each channel
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            switch (Channels[i].ChanType)
            {
            case DIG:
                myfile.print("DIGI");
                break;
            case DIG_PWM:
                myfile.print("DIGP");
                break;
            case CAN_DIGITAL:
                myfile.print("CAND");
                break;
            case CAN_PWM:
                myfile.print("CANP");
                break;
            default:
                break;
            }
            myfile.print(",");
            myfile.print(Channels[i].Enabled);

            myfile.print(",");
            myfile.print(Channels[i].CurrentValue);

            myfile.print(",");
            myfile.print(Channels[i].CurrentThresholdHigh);

            myfile.print(",");
            myfile.print(Channels[i].CurrentThresholdLow);

            myfile.print(",");
            myfile.print(Channels[i].PWMSetDuty);

            myfile.print(",");
            myfile.print(Channels[i].MultiChannel);

            myfile.print(",");
            myfile.print(Channels[i].GroupNumber);

            myfile.print(",");
            myfile.print(Channels[i].ErrorFlags, HEX);

            // Print a separating comma unless we're on the last channel
            if (i < NUM_CHANNELS - 1)
            {
                myfile.print(",");
            }
        }

        myfile.println();
        myfile.flush();

        // Reached the length limit for this file. Create a new one
        if (lineCount == 30)//StorageParams.MaxLogLength)
        {
            InitialiseSD();
        }
        lineCount++;
    }
    else
    {
        // Whatever we do next, we should close the current file
        if (!UndervoltageLatch)
        {
            myfile.flush();
            myfile.close();
            SD.end();

            // Latch this condition in case we find ourselves back here (probably cranking - sufficient voltage to run, insufficient voltage to log).
            UndervoltageLatch = true;
        }

        // Undervoltage condition. Set SD card flag OK to false to ensure we come back here and initialise a new file when we have sufficient voltage
        if (SystemParams.ErrorFlags & UNDERVOLTGAGE)
        {
            SDCardOK = false;
        }
        else
        {
            // We're back to normal operating voltage. Start a new file
            InitialiseSD();
        }
    }
}

void SleepSD()
{
    myfile.flush();
    myfile.close();
    SD.end();
}
