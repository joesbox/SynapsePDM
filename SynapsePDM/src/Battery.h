/*  Battery.h Battery charging variables, functions and data handling.
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

#ifndef Battery_H
#define Battery_H

#include <Globals.h>
#include <SparkFunBQ27441.h>

/// @brief Battery capacity in mAh
const uint16_t BATTERY_CAPACITY = 2000;

/// @brief Lowest operational voltage in mV
const uint16_t TERMINATE_VOLTAGE = 3000;

/// @brief Termination current. T4056 uses a C/10 Charge Termination. With a 1.2K current setting resistor, this would be 100mA.
const uint16_t TAPER_CURRENT = 100;

/// @brief Battery state of charge (%)
extern int SOC;

/// @brief Battery state of health (%)
extern int SOH;

/// @brief Initialises battery SOC reading
void InitialiseBattery();

/// @brief Debug battery status
void printBatteryStats();

/// @brief Manages the backup battery
void ManageBattery();

#endif