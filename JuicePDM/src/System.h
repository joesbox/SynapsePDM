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

#define VTEMP 760
#define AVG_SLOPE 2500
#define VREFINT 1210

/* Analog read resolution */
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096

/// @brief System parameters structure
struct __attribute__((packed)) SystemParameters
{
    int32_t SystemTemperature; // Internal system (STM32 processor) temperature
    uint8_t CANResEnabled;     // CAN bus termination resistor enabled
    float VBatt;               // Battery supply voltage
    float SystemCurrent;       // Total current draw for all enabled channels
    uint8_t ErrorFlags;        // Bitmask for system error flags
    int CANAddress;            // CAN Bus address
};

/// @brief System parameters
extern SystemParameters SystemParams;

/// @brief CRC check failed flag
extern bool CRCValid;

/// @brief SD card OK flag
extern bool SDCardOK;

/// @brief Power state. 0 = Run, 1 = prepare for sleep, 2 = sleeping, 3 = Ignition wake, 4 = IMU wake
extern uint8_t PowerState;

/// @brief Wake up call back for the ignition input pin
void IgnitionWake();

/// @brief Wake up call back for the IMU int 1 pin
void IMUWake1();

/// @brief Wake up call back for the IMU int 2 pin
void IMUWake2();

/// @brief Initialise system I/O and sleep functions
void InitialiseSystem();

/// @brief Updates the system parameters
void UpdateSystem();

/// @brief Reads the internal STM32 temp sensor
/// @param VRef Voltage reference
/// @return Temperature in celcius
static int32_t readTempSensor(int32_t VRef);

/// @brief Reads the internal voltage reference
/// @return Internal voltage reference valiue
static int32_t readVref();

#endif