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
#include <ChannelConfig.h>
#include <System.h>
#include <CRC32.h>
#include <EEPROM.h>
#include <STM32SD.h>
#include <M95640R.h>
#include <GSM.h>
#include <CircularBuffer.hpp>
#include <stm32f446xx.h>
#include <OutputHandler.h>

// SPI clock speed for the EEPROM
#define EEPROM_SPI_SPEED 4000000

extern long startMillis;
extern long endMillis;

/// @brief Storage parameters structure
struct __attribute__((packed)) StorageParameters
{
  char LogFileNames[10][24]; // List of currently stored log files
  uint32_t MaxLogLength;     // Max number of log lines
  uint8_t LogFrequency;      // Log frequency in Hz.
  uint8_t Reserved[32];      // Reserved for future use
};

/// @brief Storage parameters
extern StorageParameters StorageParams;

/// @brief  Storage config union for reading and writing from and to EEPROM storage
union StorageConfigUnion
{
  StorageParameters data;
  byte dataBytes[sizeof(StorageParameters)];
};

/// @brief Config storage union for storage data
extern StorageConfigUnion StorageConfigData;

/// @brief Log file object
extern File dataFile;

/// @brief Current log file name in use
extern char fileName[];

/// @brief Log file header
extern char fileHeader[];

/// @brief Accumulative bytes stored in a given log file
extern uint32_t BytesStored;

/// @brief SD card was initialised at power on
extern bool SDInit;

/// @brief Flag to latch undervoltage condition. Ensures clean-up and initialise is done only once upon re-establishing power.
extern bool UndervoltageLatch;

/// @brief Storage CRC check failed flag
extern bool StorageCRCValid;

/// @brief Analogue input config CRC check failed flag
extern bool AnalogueCRCValid;

/// @brief Saves the channel config data to EEPROM along with a calculated CRC
void SaveChannelConfig();

/// @brief Loads the config data from EEPROM storage
/// @return True if the CRC check was successful
bool LoadChannelConfig();

/// @brief Saves the system config data to EEPROM along with a calculated CRC
void SaveSystemConfig();

/// @brief Loads the system config data from EEPROM storage
/// @return True if the CRC check was successful
bool LoadSystemConfig();

/// @brief Saves the storage config data to EEPROM along with a calculated CRC
void SaveStorageConfig();

/// @brief Loads the storage config data from EEPROM storage
/// @return True if the CRC check was successful
bool LoadStorageConfig();

/// @brief Saves the analogue input config data to EEPROM along with a calculated CRC
void SaveAnalogueConfig();

/// @brief Loads the analogue input config data from EEPROM storage
/// @return True if the CRC check was successful
bool LoadAnalogueConfig();

/// @brief Inititalise storage data to known values
void InitialiseStorageData();

/// @brief Initialise the EEPROM storage
void CleanEEPROM();

/// @brief Initialises SD datalogging
void InitialiseSD();

/// @brief Logs current system and channel data to the SD card
void LogData();

/// @brief End the SD logging
void SleepSD();

/// @brief Closes the current SD file and ends the SD session
void CloseSDFile();

#endif