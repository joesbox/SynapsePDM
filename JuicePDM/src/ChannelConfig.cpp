/*  Channel.cpp The channel class defines all channel related variables and functions.
    Specifically applies to the Infineon BTS50010 High-Side Driver
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
#include "ChannelConfig.h"

ChannelConfig::ChannelConfig()
{
    pinMode(ControlPin, OUTPUT);
}


/// @brief Sets the output of the channel and calcualtes the current
/// @return Current value in amps
float ChannelConfig::SetOutput()
{
    float current = 0.0f;
    if (Enabled) // Channel is enabled
    {
        // Determine channel type. Averaging used for PWM
        switch (ChanType)
        {
        case DIG_ACT_LOW:
        case DIG_ACT_HIGH:
        case CAN_DIGITAL:
            digitalWrite(ControlPin, LOW);
            break;

        case DIG_ACT_LOW_PWM:
        case DIG_ACT_HIGH_PWM:
        case CAN_PWM:
            int duty = PWM_PERIOD * PWMSetDuty / 100;
            
            if (dutyPeriod >= duty)
            {
                // Duty period met or exceeded. Turn channel off
                digitalWrite(ControlPin, HIGH);             
                dutyPeriod = 0;
            }
            else
            {
                // Elapsed time is within the duty period. Turn channel on
                digitalWrite(ControlPin, LOW);      
                if (turnOnDelay > TURN_ON_DELAY)
                {
                    float iSense = analogRead(CurrentSensePin);


                    turnOnDelay = 0;
                }
            }
            break;

        default:
            break;
        }
    }
    else
    {
        digitalWrite(ControlPin, HIGH); // Disable output. HSD is active low.
        dutyPeriod = 0;
        turnOnDelay = 0;
        turnOffDelay = 0;
    }

    return current;
}

void ChannelConfig::SetDuty(uint8_t percentage)
{
    if (percentage > 100)
    {
        percentage = 100;
    }

    PWMSetDuty = percentage;
}
