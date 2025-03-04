/*  System.h System variables, functions and system wide data handling.
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
  uint16_t ErrorFlags;       // Bitmask for system error flags
  int CANAddress;            // CAN Bus address
  uint32_t IMUwakeWindow;    // Wake window for the IMU to determine if something needs to be done or go back to sleep
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

/// @brief Wake up call back for the IMU
void IMUWake();

/// @brief Initialise system I/O and sleep functions
void InitialiseSystem();

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