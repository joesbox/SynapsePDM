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
#include <ADC.h>
#include <Globals.h>
#include <TeensyTimerTool.h>
#include <System.h>
#include <FastLED.h>

using namespace TeensyTimerTool;

// Max frequency specified by the BTS50010A is 200Hz. Assuming 180Hz as a safe limit, an 8-bit PWM value represents aaprox. 22µs per count
#define PWM_COUNT_INTERVAL 22

// How many current sense samples to collect to calculate a mean
#define ANALOG_READ_SAMPLES 8

// 8-Bit PWM value for taking PWM analog readings. BTS50010-1LUA max turn on delay is 190µs. Assuming 180Hz as defined above, we need at least 190µs while the channel is on before taking a reading.
// An 8-bit value of 10 represents approx. 220µs
#define ANALOG_PWM_READ_INTERVAL 10

// Interval timer used to control PWM outputs and analog read back
extern IntervalTimer myTimer;

// RGB LEDs
extern CRGB leds[NUM_CHANNELS];

// Rainbow scroll
extern CRGB Scroll(int pos);

// Toggle error LED
extern uint8_t toggle[NUM_CHANNELS];

// Setup interrupts and analog read timers
void InitialiseOutputs();

// Set PWM or digital outputs
void UpdateOutputs();

// Channel ISRs which fire on the rising edge of an output
void CH1_ISR();
void CH2_ISR();
void CH3_ISR();
void CH4_ISR();
void CH5_ISR();
void CH6_ISR();

// Analog read function. Applies to PWM channels only.
void ReadPWMAnalogs();

// Analog read function. Applies to digital channels only.
void ReadDigitalAnalogs();

// Calculate channel current in amps
void CalculateAnalogs();

// Initialise the RGB LEDs
void InitialiseLEDs();

/// @brief  Update the RGB LEDs
void UpdateLEDs();

/// @brief Interval timer callback to control PWM outputs
void OutputTimer();

#endif
