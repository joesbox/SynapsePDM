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
#include <SoftPWM.h>
#include <ADC.h>
#include <IntervalTimer.h>
#include <Globals.h>
#include <TeensyTimerTool.h>

using namespace TeensyTimerTool;

// Checks output status flags set by each channel ISR. If the appropriate time has passed since turning on 
// the output (250µs for the BTS50010. See the datasheet), the analog reading from the current sense pin can be read
extern PeriodicTimer analogPWMReadTImer;

// Digital read timer reads the current sense pin associated with the channel after the maximum turn on delay for the HSD
extern PeriodicTimer analogDigitalReadTImer;

// Calculates real (ampere) current values for each channel, taking into account configured calibrations
extern PeriodicTimer calculateAnalogsTimer;

// Run channels at their set PWM or output state
void Run();

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

#endif
