/*  System.cpp System variables, functions and system wide data handling.
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

#include "System.h"

SystemParameters SystemParams;

bool CRCValid;
bool SDCardOK;

void UpdateSystem()
{
    // Get system temperature
    SystemParams.SystemTemperature = tempmonGetTemp();

    // Calculate battery voltage
    SystemParams.VBatt = analogRead(VBATT_ANALOG_PIN) * 0.0154f;

    // Calculate system current draw
    SystemParams.SystemCurrent = 0.0f;
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        SystemParams.SystemCurrent += Channels[i].CurrentValue;
    }

    // Check system temperature limit
    if (SystemParams.SystemTemperature > SYSTEM_TEMP_LIMIT)
    {
        SystemParams.ErrorFlags |= OVERTEMP;
    }
    else
    {
        SystemParams.ErrorFlags = SystemParams.ErrorFlags & ~OVERTEMP;
    }

    // Check battery voltage
    if (SystemParams.VBatt <= LOGGING_VBATT_THRESHOLD)
    {
        SystemParams.ErrorFlags |= UNDERVOLTGAGE;
    }
    else
    {
        SystemParams.ErrorFlags = SystemParams.ErrorFlags & ~UNDERVOLTGAGE;
    }

    // Check current limit
    if (SystemParams.SystemCurrent > SYSTEM_CURRENT_MAX)
    {
        SystemParams.ErrorFlags |= OVERCURRENT;
    }
    else
    {
        SystemParams.ErrorFlags = SystemParams.ErrorFlags & ~OVERCURRENT;
    }

    // Check CRC
    if (!CRCValid)
    {
        SystemParams.ErrorFlags |= CRC_CHECK_FAILED;
    }
    else
    {
        SystemParams.ErrorFlags = SystemParams.ErrorFlags & ~CRC_CHECK_FAILED;
    }

    // Check SD card status
    if (SDCardOK)
    {
        SystemParams.ErrorFlags |= SD_CARD_ERROR;
    }
    else
    {
        SystemParams.ErrorFlags = SystemParams.ErrorFlags & ~SD_CARD_ERROR;
    }
}