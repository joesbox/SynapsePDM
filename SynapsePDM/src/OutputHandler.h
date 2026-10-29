/*  OutputHandler.h Output handler deals with channel output control.
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

#ifndef OutputHandler_H
#define OutputHandler_H

#include <Arduino.h>
#include <Globals.h>
#include <System.h>

#define ANALOG_READ_SAMPLES 100
// The current-sense output loses a roughly fixed slice of every PWM pulse, so the
// unsynchronised ADC average reads proportionally low as duty falls. Measured on all
// 14 channels at 1/8/16 A: I = KILIS slope * raw * D / (D - PWM_SENSE_LAG_DUTY).
#define PWM_SENSE_LAG_DUTY 2.8F
#define PWM_SENSE_MIN_EFFECTIVE_DUTY 1.0F
// PWM over-current protection compares the estimated on-phase current (average / duty)
// against the threshold, so low-duty faults are not masked by averaging. Below this
// sampled duty the estimate is too noise-sensitive and the average is used instead.
// During the inrush window and soft-start ramps the average is always used.
#define PWM_ONPHASE_MIN_DUTY 5
#define V_REF 3.294F  // Reference voltage for ADC
#define R_IS 1000.0F  // Sense resistor value in ohms (1kΩ)
#define ADCres 4095 // 12-bit ADC resolution

#define PWM_CURRENT_FILTER_ALPHA 0.20f
#define PWM_CURRENT_ZERO_SNAP_AMPS 0.05f

/// @brief Setup interrupts and analog read timers
void InitialiseOutputs();

/// @brief Put outputs to sleep (disable DMA and timers)
void SleepOutputs();

/// @brief Setup GPIO for outputs. Push-pull, no pullups.
void setupGPIO();

/// @brief Configure DMA. Memory -> peripheral to set BSRR.
void configureDMA();

/// @brief Configure two timers to trigger DMA.
void configureTimer();

/// @brief Update duty cycle
/// @param pinIndex Pin index
/// @param dutyCycle Duty cycle in percentage (0 - 100)
void updatePWMDutyCycle(uint8_t pinIndex, uint8_t dutyCycle);

/// @brief Set PWM or digital outputs
void UpdateOutputs();

/// @brief Turn all outputs off
void OutputsOff();

/// @brief Force all outputs off until explicitly cleared.
void SetOutputsInhibited(bool inhibited);

/// @brief Returns true while outputs are globally inhibited.
bool AreOutputsInhibited();

/// @brief Returns true when a channel is temporarily shed by thermal protection.
bool IsChannelThermallyShed(uint8_t channelIndex);

/// @brief Returns the effective live enabled state after runtime interlocks.
bool IsChannelEffectivelyEnabled(uint8_t channelIndex);

/// @brief Output timer handler
void OutputTimer();

#endif
