/* Globals.h Global vars, objects and defines.
 Copyright (c) 2023 Joe Mann.  All right reserved.
 */

#ifndef Globals_H
#define Globals_H

#include <Arduino.h>
#include <ChannelConfig.h>
#include <FlexCan_T4.h>

#define NUM_CHANNELS 6

/// @brief Channel configurations
ChannelConfig Channels[NUM_CHANNELS];


/// @brief FlexCAN CAN bus interface on CAN 1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

#endif