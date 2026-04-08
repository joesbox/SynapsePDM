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
#include <IWatchdog.h>

#define VTEMP 760
#define AVG_SLOPE 2500
#define VREFINT 1210
#define RTC_MIN_VALID_YEAR 24
#define RTC_MIN_FULL_YEAR (2000 + RTC_MIN_VALID_YEAR)
#define RTC_MAX_FULL_YEAR 2099
#define TZ_MIN_OFFSET_MINUTES -720
#define TZ_MAX_OFFSET_MINUTES 840
#define TZ_MAX_DST_OFFSET_MINUTES 180
#define TZ_RULE_WEEK_LAST 5
#define TZ_RULE_DAY_MASK 0x1F
#define TZ_RULE_FIXED_DATE_FLAG 0x80

/* Analog read resolution */
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096

enum ThermalProtectionStage
{
  THERMAL_PROTECTION_NONE = 0,
  THERMAL_PROTECTION_WARNING = 1,
  THERMAL_PROTECTION_ERROR = 2
};

/// @brief Recurring time zone and DST rule.
struct __attribute__((packed)) TimeZoneRule
{
  int16_t StandardOffsetMinutes; // Base UTC offset in minutes
  int16_t DSTOffsetMinutes;      // Additional offset during DST in minutes
  uint8_t DSTEnabled;            // 0 = disabled, 1 = enabled
  uint8_t DSTStartMonth;         // Month of DST start transition
  uint8_t DSTStartWeek;          // 1-4 = nth week, 5 = last week, 0x80 | day for fixed date rules
  uint8_t DSTStartDayOfWeek;     // 0 = Sunday, 6 = Saturday. Ignored for fixed date rules
  uint8_t DSTStartHour;          // Local transition hour
  uint8_t DSTStartMinute;        // Local transition minute
  uint8_t DSTEndMonth;           // Month of DST end transition
  uint8_t DSTEndWeek;            // 1-4 = nth week, 5 = last week, 0x80 | day for fixed date rules
  uint8_t DSTEndDayOfWeek;       // 0 = Sunday, 6 = Saturday. Ignored for fixed date rules
  uint8_t DSTEndHour;            // Local transition hour
  uint8_t DSTEndMinute;          // Local transition minute
};

/// @brief System parameters structure
struct __attribute__((packed)) SystemParameters
{
  uint8_t CANResEnabled;           // CAN bus termination resistor enabled. 0 = disabled, 1 = enabled
  uint8_t SystemCurrentLimit;      // System current limit in amps
  uint16_t ChannelDataCANID;       // Channel data CAN ID. Response is this ID + 1.
  uint16_t SystemDataCANID;        // System status data CAN ID. Two messages are sent, the second is on the next ID.
  uint16_t SystemConfigDataCANID;  // System config data CAN ID.
  uint16_t ChannelConfigDataCANID; // Configuration data CAN ID. Message on this ID is basic control. Subsequent two messages for further channel configuration.
  uint32_t IMUwakeWindow;          // Wake window for the IMU to determine if something needs to be done or go back to sleep
  uint8_t MotionDeadTime;          // Time in minutes to ignore motion after wake
  uint8_t SpeedUnitPref;           // Speed units. 0 = KPH, 1 = MPH
  uint8_t DistanceUnitPref;        // Distance units. 0 = Metric (m), 1 = Imperial (ft)
  uint8_t AllowData;               // Allow mobile data
  uint8_t AllowGPS;                // Allow GPS
  uint8_t AllowMotionDetect;       // Allow motion detection wake
  TimeZoneRule TimeZone;           // Stored time zone and DST rule received from Cortex
  uint8_t DSTActive;               // Current DST state applied to the RTC
  uint8_t Reserved[14];            // Reserved for future use
};

/// @brief System runtime data structure
struct __attribute__((packed)) SystemRuntime
{
  int32_t SystemTemperature;
  float SIMModuleTemp;
  float IMUTemp;
  float VBatt;
  float SystemCurrent;
  uint16_t ErrorFlags;
};

/// @brief System parameters
extern SystemParameters SystemParams;

/// @brief System runtime parameters
extern SystemRuntime SystemRuntimeParams;

/// @brief Active thermal protection stage derived from CPU and IMU temperature.
extern volatile ThermalProtectionStage ActiveThermalProtectionStage;

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

/// @brief Validate whether a full date/time can be applied to the RTC.
/// @param fullYear Four-digit year.
/// @param month Month value in range 1-12.
/// @param day Day value in range 1-31.
/// @param hour Hour value in range 0-23.
/// @param minute Minute value in range 0-59.
/// @param second Second value in range 0-59.
/// @return True when all values are within supported ranges.
bool IsValidRtcDateTime(uint16_t fullYear, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);

/// @brief Check whether the RTC currently contains a usable date/time for logging.
/// @return True when the RTC has been set and the stored year is valid.
bool HasUsableRtcTime();

/// @brief Apply a validated date/time to the RTC and start logging support if needed.
/// @param fullYear Four-digit year.
/// @param month Month value in range 1-12.
/// @param day Day value in range 1-31.
/// @param hour Hour value in range 0-23.
/// @param minute Minute value in range 0-59.
/// @param second Second value in range 0-59.
/// @return True when the RTC was updated.
bool ApplyRtcDateTime(uint16_t fullYear, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);

/// @brief Apply a UTC date/time to the RTC after converting using the stored time zone rule.
/// @param fullYear Four-digit UTC year.
/// @param month UTC month value in range 1-12.
/// @param day UTC day value in range 1-31.
/// @param hour UTC hour value in range 0-23.
/// @param minute UTC minute value in range 0-59.
/// @param second UTC second value in range 0-59.
/// @return True when the RTC was updated or already matches the converted local time.
bool ApplyUtcRtcDateTime(uint16_t fullYear, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);

/// @brief Clamp or clear an invalid time zone rule received from storage or Cortex.
/// @param rule Rule to sanitize in place.
void SanitizeTimeZoneRule(TimeZoneRule *rule);

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