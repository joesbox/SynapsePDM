#ifndef ChannelConfig_H
#define ChannelConfig_H

#include <Arduino.h>

enum ChannelType {
  DIG_ACT_LOW,                      // Digital input, active low
  DIG_ACT_HIGH,                     // Digital input, active high
  DIG_ACT_LOW_PWM,                  // Digital input, active low, PWM output
  DIG_ACT_HIGH_PWM,                 // Digital input, active high, PWM output
  CAN                               // CAN bus controlled output
};

class ChannelConfig {
  public:
    ChannelConfig();
    String ChannelName;             // Channel Name
    ChannelType ChanType;           // Channel type
    float CurrentLimitHigh;         // Absolute current limit high
    float CurrentLimitLow;          // Absolute current limit low
    float CurrentThresholdHigh;     // Turn off threshold high
    float CurrentThresholdLow;      // Turn off threshold low
    bool Retry;                     // Retry after current threshold reached
    byte RetryCount;                // Number of retries
    float RetryDelay;               // Retry delay in seconds
};

#endif