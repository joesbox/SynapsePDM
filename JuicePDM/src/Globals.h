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

// Delay (microseconds) before makking an analog reading

#define ANALOG_DELAY 100 

// Microsecond representation of a CPU tick
#define CPU_TICK_MICROS (1E6/F_CPU)

// Interval in microseconds for taking PWM analog readings
#define ANALOG_PWM_READ_INTERVAL 50

// Interval in microseconds for taking PWM analog readings. Must be frequent enough to capture over or under current events.
#define ANALOG_DIGITAL_READ_INTERVAL 20000

// Interval in microseconds for taking PWM analog readings. Should be the same as the read interval or greater.
#define ANALOG_CALCULATION_INTERVAL 20000

// Maximum raw current sense value whereby a fault has been detected by the HSD (short to Vs, short to GND or overtemperature)
#define CURRENT_SENSE_FAULT 1024

// Maximum per-channel current supported by hardware. No channel can exceed this limit.
#define CURRENT_MAX 13000

// Default current sense ratio as specified by the BTS50010 datasheet
#define DEFAULT_DK_VALUE 38000

// Main task timer intervals (milliseconds)
#define TASK_1_INTERVAL 10
#define TASK_2_INTERVAL 50

// Unused pin that can be used to debug analog read timings which are critical to obtaining correct current measurements on PWM channels
#define ANALOG_READ_DEBUG_PIN 20

// Debug flag
#define DEBUG

// Channel digital output pins
const uint8_t channelOutputPins[NUM_CHANNELS] = {5, 4, 3, 2, 1, 0};

// Channel analog current sense pins
const uint8_t channelCurrentSensePins[NUM_CHANNELS] = {A1, A10, A17, A16, A15, A14};  

// Timers for main tasks
extern elapsedMillis task1;
extern elapsedMillis task2;

// Channel configurations
extern ChannelConfig Channels[NUM_CHANNELS];

// Initialise global data to known defaults
void InititalizeData();

#endif