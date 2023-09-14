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

ConfigUnion ConfigData;
CRC32 crc;
uint32_t EEPROMindex;
FsFile myfile;
String fileName;
SdFs SD;
int BytesStored;

void SaveConfig()
{
    EEPROMindex = 0;
    // Copy current channel info to storage structure
    memcpy(&ConfigData.data.channelConfigStored, &Channels, sizeof(Channels));

    // Calculate stored config bytes CRC
    uint32_t checksum = CRC32::calculate(ConfigData.dataBytes, sizeof(ConfigStruct));

    // Store data and CRC value
    EEPROM.put(EEPROMindex, ConfigData.dataBytes);
    EEPROMindex += sizeof(ConfigStruct);
    EEPROM.put(EEPROMindex, checksum);

    // Reset EEPROM index
    EEPROMindex = 0;
}

bool LoadConfig()
{
    // Set valid CRC flag to false
    bool validCRC = false;

    // Reset index
    EEPROMindex = 0;

    // Reset CRC result
    uint32_t result = 0;

    // Read data and CRC value
    EEPROM.get(EEPROMindex, ConfigData.dataBytes);
    EEPROMindex += sizeof(ConfigStruct);
    EEPROM.get(EEPROMindex, result);

    // Calculate read config bytes CRC
    uint32_t checksum = CRC32::calculate(ConfigData.dataBytes, sizeof(ConfigStruct));

    // Check stored CRC vs calculated CRC
    if (result == checksum)
    {
        validCRC = true;
    }

    // Reset EEPROM index
    EEPROMindex = 0;

    return validCRC;
}

void InitialiseSD()
{
    // If we can't see a card, don't proceed to initilisation
    if (!SDCardOK)
    {
        // Teensy 4.1 DMA.
        SDCardOK = SD.begin(SdioConfig(DMA_SDIO));
    }

    // Card present, continue
    if (SDCardOK)
    {
        String yearStr = year();
        String monthStr = month();
        String dayStr = day();
        String hourStr = hour();
        String minuteStr = minute();
        String secondStr = second();

        fileName = yearStr + "_" + monthStr + "_" + dayStr + "_" + hourStr + "_" + minuteStr + "_" + secondStr + ".csv";
        String fileHeader = "Date,Time,System Temp,System Voltage,System Current,Error Flags,";

        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            fileHeader = fileHeader + "Channel Type,Enabled,Current Value,Current Threshold High,Current Threshold Low,PWM,Multi-Channel,Group Number,Channel Error Flags,";
        }
        int length = fileHeader.length();
        fileHeader.remove(length - 1);

        // Create new file
        SDCardOK = myfile.open(fileName.c_str(), O_CREAT | O_WRITE);

        if (SDCardOK)
        {
            myfile.println(fileHeader);
            myfile.close();
        }
        BytesStored = 0;
        Serial.println("SD Init called");
    }
}

void LogData()
{
    // Check undervoltage flag hasn't been set and that an SD card was detected
    if (!(SystemParams.ErrorFlags & UNDERVOLTGAGE) && SDCardOK)
    {
        int start = millis();
        // Create date time stamp string
        String yearStr = year();
        String monthStr = month();
        String dayStr = day();
        String hourStr = hour();
        String minuteStr = minute();
        String secondStr = second();
        String logEntry = yearStr + "-" + monthStr + "-" + dayStr + "," + hourStr + ":" + minuteStr + ":" + secondStr;

        // Add system parameters
        logEntry = logEntry + "," + SystemParams.SystemTemperature;
        logEntry = logEntry + "," + SystemParams.VBatt;
        logEntry = logEntry + "," + SystemParams.SystemCurrent;
        logEntry = logEntry + "," + SystemParams.ErrorFlags;
        logEntry = logEntry + ",";

        // Add info for each channel
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            switch (Channels[i].ChanType)
            {
            case DIG:
                logEntry = logEntry + "DAH";
                break;
            case DIG_PWM:
                logEntry = logEntry + "DAHP";
                break;
            case CAN_DIGITAL:
                logEntry = logEntry + "CAND";
                break;
            case CAN_PWM:
                logEntry = logEntry + "CANP";
                break;
            default:
                break;
            }
            logEntry = logEntry + ",";
            logEntry = logEntry + Channels[i].Enabled + ",";
            logEntry = logEntry + Channels[i].CurrentValue + ",";
            logEntry = logEntry + Channels[i].CurrentThresholdHigh + ",";
            logEntry = logEntry + Channels[i].CurrentThresholdLow + ",";
            logEntry = logEntry + Channels[i].PWMSetDuty + ",";
            logEntry = logEntry + Channels[i].MultiChannel + ",";
            logEntry = logEntry + Channels[i].GroupNumber + ",";
            logEntry = logEntry + Channels[i].ErrorFlags + ",";
        }

        // Trim the last comma from the entry
        int length = logEntry.length();
        logEntry.remove(length - 1);

        // Write the log entry to the current file
        if (myfile.open(fileName.c_str(), O_APPEND | O_WRITE))
        {
            BytesStored += myfile.println(logEntry);
            myfile.close();
        }
        else
        {
            SDCardOK = false;
            Serial.println("Open file failed.");
        }

        // If we've gone over the max log file size, start a new file
        if (BytesStored > MAX_LOGFILE_SIZE)
        {
            Serial.print(BytesStored);
            InitialiseSD();
        }

        int finish = millis() - start;
        if (finish > TASK_3_INTERVAL)
        {
            Serial.println(finish);
        }
    }
    else
    {
        // Make sure the file is closed if we're in a low voltage state (probably powering off or cranking)
        myfile.close();
    }
}