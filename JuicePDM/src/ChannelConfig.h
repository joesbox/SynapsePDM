/*  Channel.h The channel class defines all channel related variables and functions.
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
enum ChannelType {
  DIG_ACT_LOW,                      // Digital input, active low
  DIG_ACT_HIGH,                     // Digital input, active high
  DIG_ACT_LOW_PWM,                  // Digital input, active low, PWM output
  DIG_ACT_HIGH_PWM,                 // Digital input, active high, PWM output
  CAN_DIGITAL,                      // CAN bus controlled digital output
  CAN_PWM                           // CAN bus controlled PWM output
};

/// @brief Channel config class
class ChannelConfig {
  public:
    ChannelConfig();                // Constructor

    float SetOutput();     

    String ChannelName;             // Channel Name
    ChannelType ChanType;           // Channel type
    bool Enabled;                   // Channel enabled flag
    float CurrentLimitHigh;         // Absolute current limit high
    float CurrentLimitLow;          // Absolute current limit low
    float CurrentThresholdHigh;     // Turn off threshold high
    float CurrentThresholdLow;      // Turn off threshold low
    bool Retry;                     // Retry after current threshold reached
    uint8_t RetryCount;             // Number of retries
    float RetryDelay;               // Retry delay in seconds
    bool MultiChannel;              // Grouped with other channels. Allows higher current loads
    uint8_t GroupNumber;            // Group membership number
    uint8_t ControlPin;             // Digital uC control pin 
    uint8_t CurrentSensePin;        // Current sense input pin
};

#endif