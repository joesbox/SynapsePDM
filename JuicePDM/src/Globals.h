/*  Globals.h Global variables, definitions and functions.
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

#ifndef Globals_H
#define Globals_H

#include <Arduino.h>
#include <ChannelConfig.h>

// Number of hardware output channels
#define NUM_CHANNELS 6

// Channel digital output pins
const uint8_t channelOutputPins[NUM_CHANNELS] = {5, 4, 3, 2, 1, 0};

// Channel analog current sense pins
const uint8_t channelCurrentSensePins[NUM_CHANNELS] = {A1, A10, A17, A16, A15, A14};  

// Delay (microseconds) before makking an analog reading
#define ANALOG_DELAY 250

// Microsecond representation of a CPU tick
#define CPU_TICK_MICROS (1E6/F_CPU)

// Interval in microseconds for taking analog readings
#define ANALOG_READ_INTERVAL 50

// 8-bit value that determines the priority of the analog read timer. May be useful to tune this value.
#define ANALOG_READ_TIMER_PRIORITY 128

// Maximum per-channel current supported by hardware. No channel can exceed this limit.
#define CURRENT_MAX 13000

// Timers for main tasks
extern elapsedMillis task1;
extern elapsedMillis task2;

// Main task timer intervals (milliseconds)
#define TASK_1_INTERVAL 10
#define TASK_2_INTERVAL 50

/// @brief Channel configurations
extern ChannelConfig Channels[NUM_CHANNELS];

void InititalizeData();

#endif