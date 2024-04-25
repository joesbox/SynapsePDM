/*  SerialComms.h Serial comms variables, functions and data handling.
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

#ifndef SerialComms_H
#define SerialComms_H

#include <Arduino.h>
#include <Globals.h>
#include <Storage.h>

/// @brief Setup serial port
void InitialiseSerial();

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

