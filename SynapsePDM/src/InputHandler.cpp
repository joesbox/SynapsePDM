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

        if (AnalogueIns[i].PullDownEnable)
        {
            digitalWrite(AnalogueIns[i].PullDownPin, HIGH);

            // Internal pull-down for digital inputs prevents floating inputs on boot/resume
            if (AnalogueIns[i].IsDigital)
            {
                pinMode(AnalogueIns[i].InputPin, INPUT_PULLDOWN);
            }
            else
            {
                pinMode(AnalogueIns[i].InputPin, INPUT_ANALOG);
            }
        }
        else
        {
            digitalWrite(AnalogueIns[i].PullDownPin, LOW);
            pinMode(AnalogueIns[i].InputPin, INPUT_ANALOG);
        }

        if (AnalogueIns[i].PullUpEnable)
        {
            digitalWrite(AnalogueIns[i].PullUpPin, HIGH);

            // Internal pull-up for digital inputs prevents floating inputs on boot/resume
            if (AnalogueIns[i].IsDigital)
            {
                pinMode(AnalogueIns[i].InputPin, INPUT_PULLUP);
            }
            else
            {
                pinMode(AnalogueIns[i].InputPin, INPUT_ANALOG);
            }
        }
        else
        {
            digitalWrite(AnalogueIns[i].PullUpPin, LOW);
            pinMode(AnalogueIns[i].InputPin, INPUT_ANALOG);
        }

        if (AnalogueIns[i].IsDigital)
        {
            pinMode(AnalogueIns[i].InputPin, INPUT);
        }
        else
        {
            pinMode(AnalogueIns[i].InputPin, INPUT_ANALOG);
        }
    }
}

void HandleInputs()
{
    // Check channel type and enable for active level
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        // Find the input pin index first and what type it is
        int inputPin = -1;
        bool inputIsDigital = false;

        for (int j = 0; j < NUM_DI_CHANNELS; j++)
        {
            if (Channels[i].InputControlPin == DIchannelInputPins[j])
            {
                inputPin = j;
                inputIsDigital = true;
                break;
            }
        }

        if (inputPin == -1)
        {
            for (int j = 0; j < NUM_ANA_CHANNELS; j++)
            {
                if (Channels[i].InputControlPin == ANAchannelInputPins[j])
                {
                    inputPin = j;
                    inputIsDigital = false;
                    break;
                }
            }
        }
        switch (Channels[i].ChanType)
        {
        case DIG:
        case DIG_PWM:

            // Override takes precedence over input control pin
            if (ChannelRuntime[i].Override)
            {
                Channels[i].Enabled = true;
            }
            else
            {
                if (inputIsDigital)
                {
                    Channels[i].Enabled = digitalRead(Channels[i].InputControlPin);
                }
                else
                {
                    // Analogue input used as digital
                    if (AnalogueIns[inputPin].IsDigital)
                    {
                        if (AnalogueIns[inputPin].PullUpEnable)
                        {
                            // Active low
                            Channels[i].Enabled = !digitalRead(AnalogueIns[inputPin].InputPin);
                        }
                        else
                        {
                            // Active high
                            Channels[i].Enabled = digitalRead(AnalogueIns[inputPin].InputPin);
                        }
                    }
                }
            }

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

        case CAN_DIGITAL:
        case CAN_PWM:
            // Override takes precedence over CAN message
            if (ChannelRuntime[i].Override)
            {
                Channels[i].Enabled = true;
            }
            else
            {
                Channels[i].Enabled = CANChannelEnableFlags[i];

                // Used for inrush delay timing
                if (enabledFlags[i] != Channels[i].Enabled)
                {
                    enabledFlags[i] = Channels[i].Enabled;
                    if (Channels[i].Enabled)
                    {
                        enabledTimers[i] = millis();
                    }
                }
            }
            break;
        default:
            break;
        }

        if (!Channels[i].Enabled)
        {
            // Clear error flags on disable
            ChannelRuntime[i].ErrorFlags = 0;
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