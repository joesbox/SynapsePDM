/* Channel.h The system class defines all system related variables.
Copyright (c) 2023 Joe Mann.  All right reserved.
*/

#ifndef System_H
#define System_H

#include <Arduino.h>

/// @brief System config class
class System {
  public:
    System();                       // Constructor
    String SystemName;              // Channel Name
    float GlobalCurrentLimitHigh;   // Absolute system current limit high
    float GlobalCurrentLimitHigh;   // Absolute system current limit low
    float SystemTemp;               // System temperature
     
};
#endif