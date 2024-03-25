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

// Max frequency specified by the BTS50010A is 200Hz. This 8-bit PWM period represents aaprox. 20µs per count which translates to a PWM frequency of approximately 195Hz
#define PWM_COUNT_INTERVAL 20

// How many current sense samples to collect to calculate a mean
#define ANALOG_READ_SAMPLES 10

// BTS50010-1LUA max turn on delay is 190µs (80%), max turn off delay is 200µs (20%). The effective usable 8-bit PWM range is 21 to 235
#define MIN_PWM 21
#define MAX_PWM 235

// Polynomial terms used to calculate current
#define PTERM1 9.0829
#define PTERM2 -24.874
#define PTERM3 26.468
#define PTERM4 -9.5747
#define PCONST 1.2549

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

// Initialise the RGB LEDs
void InitialiseLEDs();

/// @brief  Update the RGB LEDs
void UpdateLEDs();

/// @brief Interval timer callback to control PWM outputs
void OutputTimer();

#endif
