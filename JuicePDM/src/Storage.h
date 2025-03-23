/*  ConfigStorage.h Functions and variables for EEPROM storage and SD data logging.
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
#include <StreamUtils.h>
#include <stm32f407xx.h>
#include <OutputHandler.h>

extern SD_HandleTypeDef hsd;

// SPI clock speed for the EEPROM
#define EEPROM_SPI_SPEED 4000000

// Block size for SD writes
#define BUFFER_SIZE 4096

// SD clock divider
#define SD_CLK_DIV 0x08

extern long startMillis;
extern long endMillis;

/// @brief Storage parameters structure
struct __attribute__((packed)) StorageParameters
{
  char LogFileNames[10][24]; // List of currently stored log files
  uint32_t MaxLogLength;     // Max number of log lines
  uint8_t LogFrequency;      // Log frequency in Hz.
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

/// @brief Inititalise storage data to known values
void InitialiseStorageData();

/// @brief Initialises SD datalogging
void InitialiseSD();

/// @brief Logs current system and channel data to the SD card
void LogData();

/// @brief End the SD logging
void SleepSD();

/// @brief Closes the current SD file and ends the SD session
void CloseSDFile();

#endif