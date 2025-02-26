/*  SerialComms.h Serial comms variables, functions and data handling.
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

#ifndef SerialComms_H
#define SerialComms_H

#include <Arduino.h>
#include <Globals.h>
#include <Storage.h>

/// @brief Setup serial port
void InitialiseSerial();

/// @brief End serial comms
void SleepComms();

/// @brief Check for incoming data
void CheckSerial();

// Serial comms header
const uint16_t SERIAL_HEADER = 0x1984;

// Serial comms header
const uint16_t SERIAL_TRAILER = 0x2024;

// Command constants
const byte COMMAND_ID_BEGIN = 'b';
const byte COMMAND_ID_CONFIM = 'c';
const byte COMMAND_ID_REQUEST = 'r';
const byte COMMAND_ID_NEWCONFIG = 'n';
const byte COMMAND_ID_SEND = 's';
const byte COMMAND_ID_CHECKSUM_FAIL = 'f';
const byte COMMAND_ID_SAVECHANGES = 'S';
const byte COMMAND_ID_FW_VER = 'v';
const byte COMMAND_ID_BUILD_DATE = 'd';

/// @brief Config storage union
extern ConfigUnion SerialConfigData;

/// @brief CRC32 calculation for serial comms
extern CRC32 crcSerial;

#endif

