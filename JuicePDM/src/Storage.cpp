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
uint32_t EEPROMindex;
File myfile;
char fileName[23];
char fileHeader[65 + (132 * NUM_CHANNELS)];
int BytesStored;
bool UndervoltageLatch;

STM32RTC &rtc2 = STM32RTC::getInstance();

void SaveConfig()
{
    // Reset EEPROM index
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
        SDCardOK = SD.begin();
        if (!SDCardOK)
        {
#ifdef DEBUG
            Serial.println("SD Begin error");
            //SD.initErrorPrint();
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
        // Filename format is: YYYY-MM-DD_HH.MM.SS.csv
        char intBuffer[4];
        itoa(rtc2.getYear(), intBuffer, 10);
        strcpy(fileName, intBuffer);
        strcat(fileName, "-");
        itoa(rtc2.getMonth(), intBuffer, 10);
        strcat(fileName, intBuffer);
        strcat(fileName, "-");
        itoa(rtc2.getDay(), intBuffer, 10);
        strcat(fileName, intBuffer);
        strcat(fileName, "_");
        itoa(rtc2.getHours(), intBuffer, 10);
        strcat(fileName, intBuffer);
        strcat(fileName, "-");
        itoa(rtc2.getMinutes(), intBuffer, 10);
        strcat(fileName, intBuffer);
        strcat(fileName, "-");
        itoa(rtc2.getSeconds(), intBuffer, 10);
        strcat(fileName, intBuffer);
        strcat(fileName, ".csv");

        // Create new file
        myfile = SD.open(fileName, FILE_WRITE);
        if (!SDCardOK)
        {
#ifdef DEBUG
            Serial.println("SD init open error");
            //SD.errorPrint(&Serial);
#endif
            
        }

        // Pre-allocate
        //SDCardOK = myfile.preAllocate(MAX_LOGFILE_SIZE);
        
        if (!SDCardOK)
        {
#ifdef DEBUG
            Serial.println("SD pre-allocate error");
            //SD.errorPrint(&Serial);
#endif
            
        }

        // Create file was succesful. Write the header.
        if (SDCardOK)
        {
            // Start the ring buffer
            //myfile.begin(&myfile);

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
        myfile.print(rtc2.getYear());
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
