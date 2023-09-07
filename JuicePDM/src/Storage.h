/*  ConfigStorage.h Functions and variables for EEPROM storage and SD data logging.
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

#ifndef Storage_H
#define Storage_H

#include <Arduino.h>
#include <Globals.h>
#include <CRC32.h>
#include <ChannelConfig.h>
#include <System.h>
#include <CRC32.h>
#include <EEPROM.h> 
#include <SD.h>

/// @brief Saves the config data to EEPROM along with a calculated CRC
void SaveConfig();

/// @brief Loads the config data from EEPROM storage
/// @return True if the CRC check was successful
bool LoadConfig();

/// @brief Initialises SD datalogging
void InitialiseSD();

/// @brief Logs current system and channel data to the SD card
void LogData();

/// @brief Configuration structure
struct __attribute__((packed)) ConfigStruct
{
    ChannelConfig channelConfigStored[NUM_CHANNELS];
    SystemParameters sysParams;
};

/// @brief  Union for reading and writing from and to EEPROM storage
union ConfigUnion
{
    ConfigStruct data;
    byte dataBytes[sizeof(ConfigStruct)];
};

/// @brief CRC-32 EEPROM checksum
extern CRC32 crc;

/// @brief Config storage union
extern ConfigUnion ConfigData;

/// @brief EEPROM read and write index
extern uint32_t EEPROMindex;

/// @brief Log file object
extern File myfile;

/// @brief Current log file name in use
extern String fileName;

/// @brief SD card object
extern Sd2Card card;

/// @brief Flag to denote if the SD card was detected
extern bool CardPresent;

#endif