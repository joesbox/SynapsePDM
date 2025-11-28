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

/// @brief Defines available channel types
enum ChannelType
{
  DIG,         // Digital input
  DIG_PWM,     // Digital input, PWM output
  ANA,         // Analogue input (threshold based)
  ANA_PWM,     // Analogue input, PWM output (scaled)
  CAN_DIGITAL, // CAN bus controlled digital output
  CAN_PWM      // CAN bus controlled PWM output
};

/// @brief Channel config structure
struct __attribute__((packed)) ChannelConfig
{
  ChannelType ChanType;       // Channel type
  uint8_t PWMSetDuty;         // Current duty set percentage (0 to 100)
  uint8_t Enabled;            // Channel enabled flag
  char ChannelName[3];        // Channel name
  volatile int AnalogRaw;     // Raw analog value. Used for calibration
  float CurrentValue;         // Active current value  
  float CurrentThresholdHigh; // Turn off threshold high
  float CurrentThresholdLow;  // Turn off threshold low (open circuit detection)
  uint8_t RetryCount;         // Number of retries
  uint32_t InrushDelay;       // Inrush delay in milliseconds
  uint8_t MultiChannel;       // Grouped with other channels. Allows higher current loads
  uint8_t GroupNumber;        // Group membership number
  int OutputControlPin;       // Digital uC control pin
  uint8_t CurrentSensePin;    // Current sense input pin
  uint8_t InputControlPin;    // Input control pin
  uint8_t ActiveHigh;         // True if input is active high
  uint8_t RunOn;              // Run-on after ignition off flag
  uint32_t RunOnTime;         // Run on time (in milliseconds)
  uint8_t ErrorFlags;         // Bitmask for channel error flags
  uint8_t Override;           // Override flag
};

#endif