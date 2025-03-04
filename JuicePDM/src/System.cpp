/*  System.cpp System variables, functions and system wide data handling.
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

#include "System.h"

SystemParameters SystemParams;

bool CRCValid;
bool SDCardOK;
uint8_t PowerState;
bool RTCSet;

void IgnitionWake()
{
    PowerState = IGNITION_WAKE;
}

void IMUWake()
{
    PowerState = IMU_WAKE;
    Serial.println("IMU INT");
}

void InitialiseSystem()
{
    // Start the low power features. Attach sleep mode interrupts
    LowPower.begin();
    LowPower.attachInterruptWakeup(IGN_INPUT, IgnitionWake, RISING, DEEP_SLEEP_MODE);
    LowPower.attachInterruptWakeup(IMU_INT1, IMUWake, RISING, DEEP_SLEEP_MODE);

    // Set power state to run
    PowerState = RUN;

    // Set the analogue read resolution
    analogReadResolution(12);

    WakeSystem();
    pinMode(PA15, OUTPUT);

    // SPI
    pinMode(CS1, OUTPUT);
    pinMode(CS2, OUTPUT);
    digitalWrite(CS1, HIGH);
    digitalWrite(CS2, HIGH);
    SPI_2.begin();

    RTCSet = false;
}

void UpdateSystem()
{
    // Get system temperature
    int32_t VRef = readVref();
    SystemParams.SystemTemperature = readTempSensor(VRef);

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
    if (!SDCardOK)
    {
        SystemParams.ErrorFlags |= SDCARD_ERROR;
    }
    else
    {
        SystemParams.ErrorFlags = SystemParams.ErrorFlags & ~SDCARD_ERROR;
    }
}

void SleepSystem()
{
    // Power down peripherals
    digitalWrite(PWR_EN_5V, LOW);
    digitalWrite(PWR_EN_3V3, LOW);
}

void WakeSystem()
{
    // Power up peripherals
    pinMode(PWR_EN_5V, OUTPUT);
    pinMode(PWR_EN_3V3, OUTPUT);
    digitalWrite(PWR_EN_5V, HIGH);
    digitalWrite(PWR_EN_3V3, HIGH);
    delay(100);
}

static int32_t readTempSensor(int32_t VRef)
{
    return (__LL_ADC_CALC_TEMPERATURE(VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
}

static int32_t readVref()
{
    return (__LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION));
}
