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

#include "Globals.h"

ChannelConfig Channels[NUM_CHANNELS];
AnalogueInputs AnalogueIns[NUM_ANA_CHANNELS];
uint32_t task1Timer;
uint32_t task2Timer;
uint32_t task3Timer;
uint32_t task4Timer;
uint32_t debugTimer;

/// @brief Inititlise global data
void InititalizeData()
{
    // Initialise channels to default values, ensure they are initially off
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        Channels[i].ChanType = DIG;
        Channels[i].Enabled = false;
        Channels[i].OutputControlPin = channelOutputPins[i];
        Channels[i].CurrentSensePin = channelCurrentSensePins[i];
        Channels[i].InputControlPin = DIchannelInputPins[0];
        Channels[i].CurrentLimitHigh = CURRENT_MAX;
        Channels[i].CurrentThresholdHigh = CURRENT_MAX;
        Channels[i].CurrentThresholdLow = 0.0;
        pinMode(Channels[i].OutputControlPin, OUTPUT);
        digitalWrite(Channels[i].OutputControlPin, LOW);        
    }

    // Initialise analogue inputs to default values
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        AnalogueIns[i].InputPin = ANAchannelInputPins[i];
        AnalogueIns[i].PullDownPin = ANAchannelInputPullDowns[i];
        AnalogueIns[i].PullUpPin = ANAchannelInputPullUps[i];
        AnalogueIns[i].PullDownEnable = false;
        AnalogueIns[i].PullUpEnable = false;
    }

    // Initialise default system data
    SystemParams.CANResEnabled = true;
    SystemParams.CANAddress = ECU_ADDR;
}