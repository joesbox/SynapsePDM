/*  System.h System variables, functions and system wide data handling.
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

#ifndef System_H
#define System_H

#include <Arduino.h>
#include <Globals.h>

/// @brief System parameters structure
struct __attribute__((packed)) SystemParameters
{
    float SystemTemperature; // Internal system (Teensy processor) temperature
    uint8_t CANResEnabled;   // CAN bus termination resistor enabled
    float VBatt;             // Battery supply voltage
    float SystemCurrent;     // Total current draw for all enabled channels
    uint8_t ErrorFlags;      // Bitmask for system error flags
    uint8_t LEDBrightness;   // RGB LED brightness
    int CANAddress;          // CAN Bus address
};

/// @brief System parameters
extern SystemParameters SystemParams;

/// @brief CRC check failed flag
extern bool CRCValid;

/// @brief SD card OK flag
extern bool SDCardOK;

/// @brief Sleep flag for igniton state
extern bool goToSleep;

/// @brief Wake up call back for the ignition input pin
void WakeUpCallBack();

/// @brief Initialise system I/O and sleep functions
void InitialiseSystem();

/// @brief Updates the system parameters
void UpdateSystem();

#endif