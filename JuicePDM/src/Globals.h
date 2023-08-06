/* Globals.h Global vars, objects and defines.
 Copyright (c) 2023 Joe Mann.  All right reserved.
 */

#ifndef Globals_H
#define Globals_H

#include <Arduino.h>
#include <EepromSecureData.h>
#include <ChannelConfig.h>
#include <FlexCan_T4.h>

/// @brief Channel configurations
ChannelConfig Channel1;
ChannelConfig Channel2;
ChannelConfig Channel3;
ChannelConfig Channel4;
ChannelConfig Channel5;
ChannelConfig Channel6;

/// @brief Secure EEPROM for channel configurations
EepromSecureData<ChannelConfig> ch1;
EepromSecureData<ChannelConfig> ch2;
EepromSecureData<ChannelConfig> ch3;
EepromSecureData<ChannelConfig> ch4;
EepromSecureData<ChannelConfig> ch5;
EepromSecureData<ChannelConfig> ch6;

/// @brief FlexCAN CAN bus interface on CAN 1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

#endif