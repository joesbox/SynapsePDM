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

SystemConfigUnion SystemConfigData;
SystemParameters SystemParams;

bool SystemCRCValid;
bool ChannelCRCValid;
bool SDCardOK;
uint8_t PowerState;
bool RTCSet;
bool DisplayBacklightInitialised = false;

void IgnitionWake()
{
    SystemClock_Config();
    PowerState = IGNITION_WAKE;
}

void IMUWake()
{
    SystemClock_Config();
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

    WakeSystem();

    // Debug pin
    pinMode(DEBUG_PIN, OUTPUT);

    // Spare I/O as inputs
    pinMode(PA8, INPUT_ANALOG);
    pinMode(PC13, INPUT_ANALOG);
    pinMode(PD10, INPUT_ANALOG);
    pinMode(PD10, INPUT_ANALOG);
    pinMode(PD11, INPUT_ANALOG);
    pinMode(PD12, INPUT_ANALOG);
    pinMode(PD13, INPUT_ANALOG);
    pinMode(PD15, INPUT_ANALOG);
    pinMode(PE6, INPUT_ANALOG);
    pinMode(PG7, INPUT_ANALOG);
    pinMode(PG8, INPUT_ANALOG);

    // SPI
    pinMode(CS1, OUTPUT);
    pinMode(CS2, OUTPUT);
    digitalWrite(CS1, HIGH);
    digitalWrite(CS2, HIGH);

    RTCSet = false;

    // I2C
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();
}

void InitialiseSystemData()
{
    // Initialise default system data
    SystemParams.CANResEnabled = 1;
    SystemParams.ChannelDataCANID = CHAN_CAN_ID;
    SystemParams.SystemDataCANID = SYS_CAN_ID;
    SystemParams.ConfigDataCANID = CONF_CAN_ID;
    SystemParams.IMUwakeWindow = DEFAULT_WW;
    SystemParams.SystemCurrentLimit = SYSTEM_CURRENT_MAX;
    SystemParams.AllowData = 1;
    SystemParams.AllowGPS = 1;
    SystemParams.SpeedUnitPref = 1;
    SystemParams.DistanceUnitPref = 1;
}

void UpdateSystem()
{
    // Get system temperature
    int32_t VRef = readVref();
    SystemParams.SystemTemperature = readTempSensor(VRef);

    // Calculate battery voltage
    SystemParams.VBatt = analogRead(VBATT_ANALOG_PIN) * 0.0039787f;

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
        SystemParams.ErrorFlags |= UNDERVOLTAGE;
    }
    else
    {
        SystemParams.ErrorFlags = SystemParams.ErrorFlags & ~UNDERVOLTAGE;
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
    if (!SystemCRCValid)
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

    // Check GPS fix
    if (!GPSFix)
    {
        SystemParams.ErrorFlags |= GPS_ERROR;
    }
    else
    {
        SystemParams.ErrorFlags = SystemParams.ErrorFlags & ~GPS_ERROR;
    }

    // Checksum status of PC communications
    switch (connectionStatus)
    {
    case 0:
    case 1:
        // Disconnected or connected successfully
        SystemParams.ErrorFlags = SystemParams.ErrorFlags & ~PC_COMMS_CHECKSUM_ERROR;
        break;
    case 2:
        SystemParams.ErrorFlags |= PC_COMMS_CHECKSUM_ERROR;
        break;
    }
}

void SleepSystem()
{     
    // Power down peripherals
    pinMode(PWR_EN_5V, OUTPUT);
    pinMode(PWR_EN_3V3, OUTPUT);
    digitalWrite(PWR_EN_5V, LOW);
    digitalWrite(PWR_EN_3V3, LOW);
}

void WakeSystem()
{    
    // Set the analogue read resolution
    analogReadResolution(12);
    analogWriteResolution(10);

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
