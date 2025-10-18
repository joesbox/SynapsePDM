/*  InputHandler.cpp Input handler deals with digital channel input status.
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

#include "InputHandler.h"

void InitialiseInputs()
{
    // Ignition inout is used for wake/sleep
    pinMode(IGN_INPUT, INPUT_PULLDOWN);

    // Digital inputs. Default to active-high
    for (int i = 0; i < NUM_DI_CHANNELS; i++)
    {
        pinMode(DIchannelInputPins[i], INPUT_PULLDOWN);
    }

    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        AnalogueIns[i].InputPin = ANAchannelInputPins[i];
        AnalogueIns[i].PullDownPin = ANAchannelInputPullDowns[i];
        AnalogueIns[i].PullUpPin = ANAchannelInputPullUps[i];

        pinMode(AnalogueIns[i].PullDownPin, OUTPUT);
        pinMode(AnalogueIns[i].PullUpPin, OUTPUT);
    }
}

void HandleInputs()
{
    // Check channel type and enable for active level
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        switch (Channels[i].ChanType)
        {
        case DIG:     
        case DIG_PWM:   
            Channels[i].Enabled = digitalRead(Channels[i].InputControlPin);

            // Used for inrush delay timing
            if (enabledFlags[i] != Channels[i].Enabled)
            {
                enabledFlags[i] = Channels[i].Enabled;
                if (Channels[i].Enabled)
                {
                    enabledTimers[i] = millis();
                }
            }
            break;
        default:
            break;
        }
    }

    // Check analogue inputs. Set pull-ups/pull-downs
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        digitalWrite(AnalogueIns[i].PullDownPin, AnalogueIns[i].PullDownEnable);
        digitalWrite(AnalogueIns[i].PullUpPin, AnalogueIns[i].PullUpEnable);
    }
}

void PullResistorSleep()
{
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        pinMode(AnalogueIns[i].PullDownPin, INPUT_ANALOG);
        pinMode(AnalogueIns[i].PullUpPin, INPUT_ANALOG);
    }
}