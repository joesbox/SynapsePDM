/*  Channel.h Channel related variables and functions.
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

#ifndef ChannelConfig_H
#define ChannelConfig_H

#include <Arduino.h>

#define MIN_DELAY_TIME_MS 100UL
#define MAX_DELAY_TIME_MS 3600000UL
#define MAX_INTERMITTENT_TIME_MS 10000UL
#define DEFAULT_CHANNEL_CURRENT_SENSE_KILIS 18407.72F
#define MIN_CHANNEL_CURRENT_SENSE_KILIS 1000.0F
#define MAX_CHANNEL_CURRENT_SENSE_KILIS 100000.0F
#define MAX_MATH_RULES_PER_CHANNEL 8

/// @brief Defines available channel types
enum ChannelType
{
  DIG,             // Digital input
  DIG_PWM,         // Digital input, PWM output
  ANA,             // Analogue input (threshold based)
  ANA_PWM,         // Analogue input, PWM output (scaled)
  CAN_DIGITAL,     // CAN bus controlled digital output
  CAN_PWM,         // CAN bus controlled PWM output
  DIG_INTERMITTENT, // Digital input, intermittent digital output
  MATH             // Structured expression-driven output
};

static_assert(DIG == 0 && DIG_PWM == 1 && ANA == 2 && ANA_PWM == 3 &&
                  CAN_DIGITAL == 4 && CAN_PWM == 5 && DIG_INTERMITTENT == 6 && MATH == 7,
              "ChannelType EEPROM and protocol values must remain stable");

enum MathOperator : uint8_t
{
  MATH_ADD = 0,
  MATH_SUBTRACT,
  MATH_MULTIPLY,
  MATH_DIVIDE,
  MATH_GREATER_THAN,
  MATH_GREATER_EQUAL,
  MATH_LESS_THAN,
  MATH_LESS_EQUAL,
  MATH_EQUAL,
  MATH_NOT_EQUAL,
  MATH_AND,
  MATH_OR,
  MATH_XOR,
  MATH_NOT,
  MATH_MIN,
  MATH_MAX,
  MATH_ABS,
  MATH_TOGGLE,
  MATH_OPERATOR_COUNT
};

enum MathOperandType : uint8_t
{
  MATH_OPERAND_CONSTANT = 0,
  MATH_OPERAND_CHANNEL,
  MATH_OPERAND_ANALOGUE_INPUT,
  MATH_OPERAND_SYSTEM_VALUE,
  MATH_OPERAND_RULE,
  MATH_OPERAND_DIGITAL_INPUT,
  MATH_OPERAND_CAN_SIGNAL,
  MATH_OPERAND_TYPE_COUNT
};

enum MathSystemValue : uint8_t
{
  MATH_SYSTEM_TEMPERATURE = 0,
  MATH_SYSTEM_SIM_TEMPERATURE,
  MATH_SYSTEM_IMU_TEMPERATURE,
  MATH_SYSTEM_BATTERY_VOLTAGE,
  MATH_SYSTEM_CURRENT,
  MATH_SYSTEM_ACCEL_X,
  MATH_SYSTEM_ACCEL_Y,
  MATH_SYSTEM_ACCEL_Z,
  MATH_SYSTEM_GYRO_X,
  MATH_SYSTEM_GYRO_Y,
  MATH_SYSTEM_GYRO_Z,
  MATH_SYSTEM_MAG_X,
  MATH_SYSTEM_MAG_Y,
  MATH_SYSTEM_MAG_Z,
  MATH_SYSTEM_VALUE_COUNT
};

enum MathOutputMode : uint8_t
{
  MATH_OUTPUT_DIGITAL = 0,
  MATH_OUTPUT_PWM = 1
};

struct __attribute__((packed)) MathOperand
{
  uint8_t Type;
  uint8_t Index;
  float Constant;
};

struct __attribute__((packed)) MathRuleConfig
{
  MathOperator Operation;
  MathOperand OperandA;
  MathOperand OperandB;
  uint8_t HysteresisEnabled;
  float OnThreshold;
  float OffThreshold;
};

static_assert(sizeof(MathRuleConfig) == 22, "MATH rule EEPROM layout must remain stable");

struct __attribute__((packed)) MathChannelConfig
{
  uint8_t RuleCount;
  uint8_t RootRule;
  uint8_t OutputMode;
  MathRuleConfig Rules[MAX_MATH_RULES_PER_CHANNEL];
};

static_assert(sizeof(MathChannelConfig) == 179, "MATH channel EEPROM layout must remain stable");

struct MathRuleRuntime
{
  float Value;
  uint8_t Error;
  uint8_t Evaluated;
};

struct MathChannelRuntime
{
  float Value;
  uint8_t Error;
  uint8_t Evaluated;
  MathRuleRuntime Rules[MAX_MATH_RULES_PER_CHANNEL];
};

enum MathRuntimeError : uint8_t
{
  MATH_ERROR_NONE = 0,
  MATH_ERROR_CONFIGURATION = 1,
  MATH_ERROR_REFERENCE = 2,
  MATH_ERROR_DEPENDENCY = 3,
  MATH_ERROR_EVALUATION = 4
};

/// @brief Defines supported output channel categories
enum ChannelCategory
{
  CHANNEL_CATEGORY_ECU_POWER,
  CHANNEL_CATEGORY_IGNITION_COILS,
  CHANNEL_CATEGORY_FUEL_PUMP,
  CHANNEL_CATEGORY_FUEL_INJECTORS,
  CHANNEL_CATEGORY_ENGINE_SENSORS_SUPPLY,
  CHANNEL_CATEGORY_DRIVE_BY_WIRE,
  CHANNEL_CATEGORY_HEADLIGHTS,
  CHANNEL_CATEGORY_BRAKE_LIGHTS,
  CHANNEL_CATEGORY_INDICATORS,
  CHANNEL_CATEGORY_HAZARD_LIGHTS,
  CHANNEL_CATEGORY_HORN,
  CHANNEL_CATEGORY_WIPERS,
  CHANNEL_CATEGORY_WASHER_PUMP,
  CHANNEL_CATEGORY_ABS_BRAKE_SYSTEM,
  CHANNEL_CATEGORY_POWER_STEERING,
  CHANNEL_CATEGORY_COOLING_FAN,
  CHANNEL_CATEGORY_OIL_COOLER_FAN,
  CHANNEL_CATEGORY_WATER_PUMP,
  CHANNEL_CATEGORY_INTERCOOLER_PUMP,
  CHANNEL_CATEGORY_TRANSMISSION_PUMP,
  CHANNEL_CATEGORY_TAIL_LIGHTS,
  CHANNEL_CATEGORY_DRL,
  CHANNEL_CATEGORY_REVERSE_LIGHTS,
  CHANNEL_CATEGORY_INTERIOR_LIGHTS,
  CHANNEL_CATEGORY_DASH_CLUSTER,
  CHANNEL_CATEGORY_GEAR_SELECTOR,
  CHANNEL_CATEGORY_HEATED_SEATS,
  CHANNEL_CATEGORY_HEATED_STEERING_WHEEL,
  CHANNEL_CATEGORY_HVAC_BLOWER,
  CHANNEL_CATEGORY_AC_CLUTCH,
  CHANNEL_CATEGORY_INFOTAINMENT,
  CHANNEL_CATEGORY_USB_ACCESSORY_POWER,
  CHANNEL_CATEGORY_DATA_LOGGER,
  CHANNEL_CATEGORY_TELEMETRY,
  CHANNEL_CATEGORY_CAMERA_SYSTEM,
  CHANNEL_CATEGORY_LAP_TIMER,
  CHANNEL_CATEGORY_COOL_SUIT_PUMP,
  CHANNEL_CATEGORY_FIRE_SUPPRESSION,
  CHANNEL_CATEGORY_RAIN_LIGHT,
  CHANNEL_CATEGORY_PIT_LIMITER,
  CHANNEL_CATEGORY_AUXILIARY,
  CHANNEL_CATEGORY_SPARE,
  CHANNEL_CATEGORY_CUSTOM,
  CHANNEL_CATEGORY_COUNT
};

/// @brief Thermal protection priority assigned to each category
enum ChannelPriority
{
  CHANNEL_PRIORITY_CRITICAL,
  CHANNEL_PRIORITY_MEDIUM,
  CHANNEL_PRIORITY_LOW
};

enum DelayedOffTriggerSource
{
  DELAYED_OFF_ASSIGNED_INPUT = 0,
  DELAYED_OFF_IGNITION_OFF = 1
};

/// @brief Channel config structure
struct __attribute__((packed)) ChannelConfig
{
  ChannelType ChanType;         // Channel type
  ChannelCategory Category;     // Output category
  uint8_t PWMSetDuty;           // Current duty set percentage (0 to 100)
  uint8_t Enabled;              // Persisted channel enable/config flag
  char ChannelName[3];          // Channel name
  float CurrentThresholdHigh;   // Turn off threshold high
  float CurrentThresholdLow;    // Turn off threshold low (open circuit detection)
  uint8_t RetryCount;           // Number of retries
  uint32_t InrushDelay;         // Inrush delay in milliseconds
  uint8_t MultiChannel;         // Grouped with other channels. Allows higher current loads
  uint8_t GroupNumber;          // Group membership number
  int OutputControlPin;         // Digital uC control pin
  uint8_t CurrentSensePin;      // Current sense input pin
  uint8_t InputControlPin;      // Input control pin
  float OnThreshold;            // On threshold (Voltage)
  float OffThreshold;           // Off threshold (Voltage)
  float ScaleMin;               // Minimum scale value (Used for PWM scaled inputs)
  float ScaleMax;               // Maximum scale value (Used for PWM scaled inputs)
  uint8_t PWMMin;               // Minimum PWM value (0-100%)
  uint8_t PWMMax;               // Maximum PWM value (0-100%)
  uint8_t ActiveHigh;           // True if input is active high
  uint8_t RunOn;                // Run-on after ignition off flag
  uint32_t RunOnTime;           // Run on time (in milliseconds)
  uint8_t SoftStart;            // Soft start enabled flag
  uint32_t SoftStartTime;       // Soft start time in milliseconds
  uint8_t SoftStop;             // Soft stop enabled flag
  uint32_t SoftStopTime;        // Soft stop time in milliseconds
  float InrushCurrentThreshold; // Inrush current threshold for soft start (in amps)
  uint32_t IntermittentOnTime;  // Intermittent on time in milliseconds
  uint32_t IntermittentOffTime; // Intermittent off time in milliseconds
  float CurrentSenseKILIS;      // Per-channel current sense ratio calibration value
  uint8_t DelayedOn;            // Delayed-on enabled flag
  uint32_t DelayedOnTime;       // Delayed-on time in milliseconds
  uint8_t DelayedOff;           // Delayed-off enabled flag
  uint32_t DelayedOffTime;      // Delayed-off time in milliseconds
  uint8_t DelayedOffTrigger;    // Delayed-off trigger source
  uint8_t Reserved[3];          // Reserved for future use
};

static_assert(sizeof(ChannelConfig) == 92, "ChannelConfig EEPROM layout must remain compatible with v0.9");

/// @brief Channel config runtime structure
struct __attribute__((packed)) ChannelConfigRuntime
{
  volatile int AnalogRaw; // Raw analog value. Used for calibration
  float CurrentValue;     // Active current value
  uint8_t ErrorFlags;     // Bitmask for channel error flags
  uint8_t Override;       // Override flag
  uint8_t Enabled;        // Live runtime enable state
};

/// @brief Clamp a persisted category to a supported value.
ChannelCategory SanitizeChannelCategory(uint8_t rawCategory);

/// @brief Returns the thermal protection priority for a category.
ChannelPriority GetChannelPriority(ChannelCategory category);

/// @brief Normalise persisted channel config values after load.
/// @return True when any persisted field was changed.
bool SanitizeChannelConfig(ChannelConfig &config);

#endif