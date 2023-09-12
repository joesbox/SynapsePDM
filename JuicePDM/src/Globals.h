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
#include <System.h>
#include <TimeLib.h>

// Number of hardware output channels
#define NUM_CHANNELS 6

// Delay (microseconds) before making an analog reading
// TODO: measure actual turn on delay and adjust this value
#define ANALOG_DELAY 100 

// Maximum PWM duty accounting for turn off delay. Above this value, a PWM channel will be set to 100% duty
#define MAX_DUTY 90        

// Min duty accounting for turn on delay. Above this value, a PWM channel will be set to 0% duty
#define MIN_DUTY 10        

// Microsecond representation of a CPU tick
#define CPU_TICK_MICROS (1E6/F_CPU)

// Interval in microseconds for taking PWM analog readings
#define ANALOG_PWM_READ_INTERVAL 20

// Interval in microseconds for taking PWM analog readings. Must be frequent enough to capture over or under current events.
#define ANALOG_DIGITAL_READ_INTERVAL 20000

// Interval in microseconds for taking PWM analog readings. Should be the same as the read interval or greater.
#define ANALOG_CALCULATION_INTERVAL 20000

// Maximum raw current sense value whereby a fault has been detected by the HSD (short to Vs, short to GND or overtemperature)
#define CURRENT_SENSE_FAULT 1024

// Maximum per-channel current supported by hardware. No channel can exceed this limit.
#define CURRENT_MAX 13.0

// Sytem current max (total). Should never reach this as each channel is independantly monitored
#define SYSTEM_CURRENT_MAX 78.0

// Default current sense ratio as specified by the BTS50010 datasheet
#define DEFAULT_DK_VALUE 38000

// Maximum log file size in bytes
#define MAX_LOGFILE_SIZE 1000000

// Main task timer intervals (milliseconds)
#define TASK_1_INTERVAL 10
#define TASK_2_INTERVAL 50
#define TASK_3_INTERVAL 100
#define TASK_4_INTERVAL 250

// Watchdog timer interval
#define WATCHDOG_INTERVAL 2500

// RGB LED serial data pin
#define RGB_PIN 9

// Default RGB LED brightness
#define DEFAULT_RGB_BRIGHTNESS 64

// Unused pin that can be used to debug analog read timings which are critical to obtaining correct current measurements on PWM channels
#define ANALOG_READ_DEBUG_PIN 20

// Debug flag
#define DEBUG

// Battery measurement analog input pin
#define VBATT_ANALOG_PIN A7

// Nominal battery voltage
#define VBATT_NOMINAL 13.8

// CAN bus termination resistor enable pin
#define CAN_BUS_RESISTOR_ENABLE 6

// Battery voltage threshold at which power loss is immenent and logging should be stopped
#define LOGGING_VBATT_THRESHOLD 9.0

// Maximum permissible system temperature
#define SYSTEM_TEMP_LIMIT 80.0

// System error bitmasks
#define OVERCURRENT 0x01
#define OVERTEMP 0x02
#define UNDERVOLTGAGE 0x04
#define CRC_CHECK_FAILED 0x08

// Channel error bitmasks
#define CHN_OVERCURRENT_RANGE 0x01
#define CHN_OVERCURRENT_LIMIT 0x02
#define CHN_UNDERCURRENT_RANGE 0x04
#define OVERTEMP_GNDSHORT 0x08
#define WATCHDOG_TIMEOUT 0x16

// ECU CAN address
#define ECU_ADDR 0x800;

// Channel digital input pins (defaults)
const uint8_t channelInputPins[NUM_CHANNELS] = {24, 25, 26, 29, 28, 27};

// Channel digital output pins
const uint8_t channelOutputPins[NUM_CHANNELS] = {5, 4, 3, 2, 1, 0};

// Channel analog current sense pins
const uint8_t channelCurrentSensePins[NUM_CHANNELS] = {A1, A10, A17, A16, A15, A14};  

// Timers for main tasks
extern elapsedMillis task1;
extern elapsedMillis task2;
extern elapsedMillis task3;
extern elapsedMillis task4;

// Channel configurations
extern ChannelConfig Channels[NUM_CHANNELS];

// Initialise global data to known defaults
void InititalizeData();

// Sync time function
time_t getTeensy3Time();

#endif