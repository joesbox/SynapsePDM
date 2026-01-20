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
  uint8_t CANResEnabled;      // CAN bus termination resistor enabled. 0 = disabled, 1 = enabled
  uint8_t SystemCurrentLimit; // System current limit in amps
  uint16_t ChannelDataCANID;  // Channel data CAN ID (transmit)
  uint16_t SystemDataCANID;   // System data CAN ID (transmit)
  uint16_t ConfigDataCANID;   // Configuration data CAN ID (receive)
  uint32_t IMUwakeWindow;     // Wake window for the IMU to determine if something needs to be done or go back to sleep
  uint8_t MotionDeadTime;     // Time in minutes to ignore motion after wake
  uint8_t SpeedUnitPref;      // Speed units. 0 = KPH, 1 = MPH
  uint8_t DistanceUnitPref;   // Distance units. 0 = Metric (m), 1 = Imperial (ft)
  uint8_t AllowData;          // Allow mobile data
  uint8_t AllowGPS;           // Allow GPS
  uint8_t AllowMotionDetect;  // Allow motion detection wake
  uint8_t Reserved[32];       // Reserved for future use
};

/// @brief System runtime data structure
struct __attribute__((packed))SystemRuntime
{
    int32_t SystemTemperature;
    float VBatt;
    float SystemCurrent;
    uint16_t ErrorFlags;
};

/// @brief System parameters
extern SystemParameters SystemParams;

/// @brief System runtime parameters
extern SystemRuntime SystemRuntimeParams;

/// @brief  System config union for reading and writing from and to EEPROM storage
union SystemConfigUnion
{
  SystemParameters data;
  byte dataBytes[sizeof(SystemParameters)];
};

/// @brief Config storage union for system data
extern SystemConfigUnion SystemConfigData;

/// @brief System CRC check failed flag
extern bool SystemCRCValid;

/// @brief Channel CRC check failed flag
extern bool ChannelCRCValid;

/// @brief SD card OK flag
extern bool SDCardOK;

/// @brief Power state. 0 = Run, 1 = prepare for sleep, 2 = sleeping, 3 = Ignition wake, 4 = IMU wake
extern volatile uint8_t PowerState;

/// @brief Flag to denote RTC has been set
extern bool RTCSet;

/// @brief Flag to latch display backlight initialisation
extern bool DisplayBacklightInitialised;

/// @brief Wake up call back for the ignition input pin
void IgnitionWake();

/// @brief Wake up call back for the IMU
void IMUWake();

/// @brief Initialise system I/O and sleep functions
void InitialiseSystem();

/// @brief Initialise system data to known
void InitialiseSystemData();

/// @brief Updates the system parameters
void UpdateSystem();

/// @brief Power down the peripheral supply rails
void SleepSystem();

/// @brief Power up the peripheral supply rails
void WakeSystem();

/// @brief Reads the internal STM32 temp sensor
/// @param VRef Voltage reference
/// @return Temperature in celcius
static int32_t readTempSensor(int32_t VRef);

/// @brief Reads the internal voltage reference
/// @return Internal voltage reference valiue
static int32_t readVref();

#endif