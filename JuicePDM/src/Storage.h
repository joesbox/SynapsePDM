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

// SD sector size
#define SD_SECTOR_SIZE 512

// Logging ring buffer capacity in bytes
#define RING_BUF_CAPACITY 2000

// SPI clock speed
#define SPI_CLOCK SD_SCK_MHZ(50)

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
extern char fileName[];

/// @brief Log file header
extern char fileHeader[];

/// @brief Accumulative bytes stored in a given log file
extern int BytesStored;

/// @brief SD card was initialised at power on
extern bool SDInit;

/// @brief Flag to latch undervoltage condition. Ensures clean-up and initialise is done only once upon re-establishing power.
extern bool UndervoltageLatch;

#endif