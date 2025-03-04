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

ConfigUnion ConfigData;
CRC32 crc;
uint16_t EEPROMindex;
File myfile;
char fileName[23];
char fileHeader[65 + (132 * NUM_CHANNELS)];
int BytesStored;
bool UndervoltageLatch;

STM32RTC &rtc2 = STM32RTC::getInstance();

M95640R EEPROMext(&SPI_2, CS1);

void SaveConfig()
{
    EEPROMext.begin(EEPROM_SPI_SPEED);

    // Reset EEPROM index
    EEPROMindex = 0;

    // Copy current channel and system info to storage structure
    memcpy(&ConfigData.data.channelConfigStored, &Channels, sizeof(Channels));
    memcpy(&ConfigData.data.sysParams, &SystemParams, sizeof(SystemParams));

    // Calculate stored config bytes CRC
    uint32_t checksum = CRC32::calculate(ConfigData.dataBytes, sizeof(ConfigStruct));

    // Store data and CRC value
    uint8_t int32Buf[4];
    int32Buf[0] = (checksum >> 24) & 0xFF;
    int32Buf[1] = (checksum >> 16) & 0xFF;
    int32Buf[2] = (checksum >> 8) & 0xFF;
    int32Buf[3] = checksum & 0xFF;

    // Bear in mind that the struct has the packed attribute and will report a different size in PlatformIO/VSCode
    for (unsigned int i = 0; i < sizeof(ConfigData.dataBytes); i++)
    {
        EEPROMext.EepromWrite(EEPROMindex, 1, &ConfigData.dataBytes[i]);
        EEPROMindex++;
    }

#ifdef DEBUG
    Serial.print("Checksum written: ");
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

bool LoadConfig()
{
    EEPROMext.begin(EEPROM_SPI_SPEED);
    // Set valid CRC flag to false
    bool validCRC = false;

    // Reset index
    EEPROMindex = 0;

    // Reset CRC result
    uint32_t result = 0;

    uint8_t int32Buf[4];

    for (unsigned int i = 0; i < sizeof(ConfigData.dataBytes); i++)
    {
        EEPROMext.EepromRead(EEPROMindex, 1, &ConfigData.dataBytes[i]);
        EEPROMindex++;
    }
    EEPROMext.EepromRead(EEPROMindex, sizeof(result), int32Buf);

    result = (int32_t(int32Buf[0]) << 24) |
             (int32_t(int32Buf[1]) << 16) |
             (int32_t(int32Buf[2]) << 8) |
             int32_t(int32Buf[3]);

#ifdef DEBUG
    Serial.print("Checksum read: ");
    Serial.print(result, HEX);
    Serial.print(", at index: ");
    Serial.println(EEPROMindex);
    Serial.print("EEPROM status register: ");
    Serial.println(EEPROMext.EepromStatus());
#endif
    // Calculate read config bytes CRC
    uint32_t checksum = CRC32::calculate(ConfigData.dataBytes, sizeof(ConfigStruct));

    // Check stored CRC vs calculated CRC
    if (result == checksum)
    {
        validCRC = true;
        // Copy channel and system info
        memcpy(&Channels, &ConfigData.data.channelConfigStored, sizeof(Channels));
        memcpy(&SystemParams, &ConfigData.data.sysParams, sizeof(SystemParams));
    }

    // Reset EEPROM index
    EEPROMindex = 0;
    EEPROMext.end();

    return validCRC;
}

void InitialiseSD()
{
    // If we can't see a card, don't proceed to initilisation
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
        // Filename format is: YY-MM-DD_HH-MM-SS.csv
        sprintf(fileName, "%02d-%02d-%02d_%02d-%02d-%02d.csv", rtc2.getYear(), rtc2.getMonth(), rtc2.getDay(), rtc2.getHours(), rtc2.getMinutes(), rtc2.getSeconds());

        // Create new file
        myfile = SD.open(fileName, FILE_WRITE);
        if (!SDCardOK)
        {
#ifdef DEBUG
            Serial.println("SD init open error");
            // SD.errorPrint(&Serial);
#endif
        }

        // Create file was succesful. Write the header.
        if (myfile)
        {
            // Start the ring buffer
            // myfile.begin(&myfile);

            // Print the file header to the buffer
            myfile.print("Date,Time,System Temp,System Voltage,System Current,Error Flags,");

            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                myfile.print("Channel Type,Enabled,Current Value,Current Threshold High,Current Threshold Low,PWM,Multi-Channel,Group Number,Channel Error Flags");

                // Print a separating comma unless we're on the last channel
                if (i != NUM_CHANNELS)
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
    }
}

void LogData()
{
    // Log if we're not in an undervoltage condition and the SD card is OK
    if (!(SystemParams.ErrorFlags & UNDERVOLTGAGE) && SDCardOK)
    {
        int start = millis();
        // Print date and time to the buffer

        // TODO: Sort the date format out with sprintf
        myfile.print(2000 + rtc2.getYear());
        myfile.print("-");
        myfile.print(rtc2.getMonth());
        myfile.print("-");
        myfile.print(rtc2.getDay());
        myfile.print(",");
        myfile.print(rtc2.getHours());
        myfile.print(":");
        myfile.print(rtc2.getMinutes());
        myfile.print(":");
        myfile.print(rtc2.getSeconds());
        myfile.print(",");

        // Add system parameters
        myfile.print(SystemParams.SystemTemperature);
        myfile.print(",");

        myfile.print(SystemParams.VBatt);
        myfile.print(",");

        myfile.print(SystemParams.SystemCurrent);
        myfile.print(",");

        myfile.print(SystemParams.ErrorFlags);
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
            myfile.print(Channels[i].ErrorFlags);

            // Print a separating comma unless we're on the last channel
            if (i != NUM_CHANNELS)
            {
                myfile.print(",");
            }
        }

        myfile.println();
        myfile.flush();

        /*// Ring buffer contains enough data for one sector and the file isn't busy. Write the contents of the buffer out.
        if ((myfile.bytesUsed() >= SD_SECTOR_SIZE) && !myfile.isBusy())
        {
            BytesStored = myfile.writeOut(SD_SECTOR_SIZE);
            // Make sure that the entire sector was written successfully
            if (SD_SECTOR_SIZE != BytesStored)
            {
                SDCardOK = false;
#ifdef DEBUG
                Serial.println("Sector size != bytes stored");
#endif
            }
        }

        // Check whether the file is full (i.e. When there is not enough room to write 1 more sector) or there was an error writing out to the file, truncate this file, close it and start a new file
        if ((myfile.dataLength() - myfile.curPosition()) < SD_SECTOR_SIZE || !SDCardOK)
        {
            // Write any RingBuf data to file. Initialise a new file.
            myfile.sync();
            myfile.truncate();
            myfile.rewind();
            myfile.close();
            myfile.sync();
#ifdef DEBUG
            Serial.println("File full.");
#endif
            InitialiseSD();
        }*/

        int finish = millis() - start;
        if (finish > TASK_3_INTERVAL)
        {
#ifdef DEBUG
            Serial.println(finish);
#endif
        }
    }
    else
    {
        // Whatever we do next, we should close the current file
        if (!UndervoltageLatch)
        {
            myfile.close();

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
    SD.end();
}
