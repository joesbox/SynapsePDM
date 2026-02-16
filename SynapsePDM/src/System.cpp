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
SystemRuntime SystemRuntimeParams;

#define CDC_TRANSMIT_QUEUE_BUFFER_PACKET_NUMBER 20

bool SystemCRCValid;
bool ChannelCRCValid;
bool SDCardOK;
volatile uint8_t PowerState;
bool RTCSet;
bool DisplayBacklightInitialised = false;

void IgnitionWake()
{
    PowerState = IGNITION_WAKING;
}

void IMUWake()
{
    if (!IMUWakeMode)
    {
        IMUWakeMode = true;
        imuWakePending = true;
    }
}

void InitialiseSystem()
{
    // Start the low power features. Attach sleep mode interrupts
    LowPower.begin();
    LowPower.attachInterruptWakeup(IMU_INT1, IMUWake, RISING, DEEP_SLEEP_MODE);
    LowPower.attachInterruptWakeup(IGN_INPUT, IgnitionWake, RISING, DEEP_SLEEP_MODE);

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

    pinMode(CHARGE_EN, OUTPUT);
    pinMode(BATT_INT, INPUT);

    digitalWrite(CHARGE_EN, HIGH); // Active low

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
    memset(&SystemParams, 0, sizeof(SystemParams));
    SystemParams.CANResEnabled = 1;
    SystemParams.ChannelDataCANID = CHAN_CAN_ID;
    SystemParams.SystemDataCANID = SYS_CAN_ID;
    SystemParams.SystemConfigDataCANID = SYS_CONFIG_CAN_ID;
    SystemParams.ChannelConfigDataCANID = CONF_CAN_ID;
    SystemParams.IMUwakeWindow = DEFAULT_WW;
    SystemParams.MotionDeadTime = DEFAULT_MOTION_DEADTIME;
    SystemParams.SystemCurrentLimit = SYSTEM_CURRENT_MAX;
    SystemParams.AllowData = 1;
    SystemParams.AllowGPS = 1;
    SystemParams.SpeedUnitPref = 1;
    SystemParams.DistanceUnitPref = 1;
    SystemParams.AllowMotionDetect = 1;
}

void UpdateSystem()
{
    // Get system temperature
    int32_t VRef = readVref();
    SystemRuntimeParams.SystemTemperature = readTempSensor(VRef);

    // Calculate battery voltage
    SystemRuntimeParams.VBatt = analogRead(VBATT_ANALOG_PIN) * 0.0039787f;

    // Calculate system current draw
    SystemRuntimeParams.SystemCurrent = 0.0f;
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        SystemRuntimeParams.SystemCurrent += ChannelRuntime[i].CurrentValue;
    }

    // Check system temperature limit
    if (SystemRuntimeParams.SystemTemperature > SYSTEM_TEMP_LIMIT)
    {
        SystemRuntimeParams.ErrorFlags |= OVERTEMP;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~OVERTEMP;
    }

    // Check battery voltage
    if (SystemRuntimeParams.VBatt <= LOGGING_VBATT_THRESHOLD)
    {
        SystemRuntimeParams.ErrorFlags |= UNDERVOLTAGE;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~UNDERVOLTAGE;
    }

    // Check current limit
    if (SystemRuntimeParams.SystemCurrent > SYSTEM_CURRENT_MAX)
    {
        SystemRuntimeParams.ErrorFlags |= OVERCURRENT;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~OVERCURRENT;
    }

    // Check CRC
    if (!SystemCRCValid)
    {
        SystemRuntimeParams.ErrorFlags |= CRC_CHECK_FAILED;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~CRC_CHECK_FAILED;
    }

    // Check SD card status
    if (!SDCardOK)
    {
        SystemRuntimeParams.ErrorFlags |= SDCARD_ERROR;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~SDCARD_ERROR;
    }

    // Check GPS fix
    if (!GPSFix && SystemParams.AllowGPS)
    {
        SystemRuntimeParams.ErrorFlags |= GPS_ERROR;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~GPS_ERROR;
    }

    // Checksum status of PC communications
    switch (connectionStatus)
    {
    default:
        // Disconnected or connected successfully
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~PC_COMMS_CHECKSUM_ERROR;
        break;
    case 8:
        SystemRuntimeParams.ErrorFlags |= PC_COMMS_CHECKSUM_ERROR;
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
    delay(20); // Allow power rails to stabilise
}

static int32_t readTempSensor(int32_t VRef)
{
    return (__LL_ADC_CALC_TEMPERATURE(VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
}

static int32_t readVref()
{
    return (__LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION));
}
