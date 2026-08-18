/*  InputHandler.h Input handler deals with digital channel input status.
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

#ifndef InputHandler_H
#define InputHandler_H

#include <Arduino.h>
#include <Globals.h>

#define ANALOG_PULL_RESISTOR_OHMS 2490.0f
#define ANALOG_SENSOR_SUPPLY_VOLTAGE 5.0f
#define ANALOG_DIGITAL_THRESHOLD_VOLTS 2.5f
#define NTC_T0_KELVIN 298.15f
#define MIN_DENOMINATOR 0.0001f

/// @brief Initialise inputs
void InitialiseInputs();

/// @brief Handles reading of inputs
void HandleInputs();

/// @brief Reset per-channel delayed on/off and source-state timing state.
void ResetChannelInputTimingStates();

/// @brief Coerces channel types to match the assigned analogue input mode.
bool SyncChannelTypeForAssignedInput(uint8_t channelIndex);

/// @brief Coerces all channels bound to an analogue input to match that input's mode.
bool SyncChannelTypesForAnalogueInput(uint8_t inputIndex);

/// @brief Clamp a persisted analogue input configuration to supported values.
void SanitizeAnalogueInputConfig(AnalogueInputs &input);

/// @brief Read an analogue input and update its live voltage and converted value.
float ReadAnalogueInputValue(int inputIndex);

/// @brief Returns the live runtime enabled state for a channel.
bool IsChannelRuntimeEnabled(uint8_t channelIndex);

/// @brief Disables all pull-up and pull-down resistor outputs
void PullResistorSleep();

#endif