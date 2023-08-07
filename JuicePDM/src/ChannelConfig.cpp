/*  Channel.cpp The channel class defines all channel related variables and functions.
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
}

/// @brief Sets the output of the channel and calcualtes the current
/// @return Current value in amps
float ChannelConfig::SetOutput()
{
    float current = 0.0f;

    // Determine channel type. Averaging used for PWM
    switch (ChanType)
    {
    case DIG_ACT_LOW:
    case DIG_ACT_HIGH:
    case CAN_DIGITAL:
        if (Enabled)        // Channel is enabled
        {
            //TODO: Write software PWM function at 100Hz and measure current
        }
        break;

    case DIG_ACT_LOW_PWM:
    case DIG_ACT_HIGH_PWM:
    case CAN_PWM:
        if (Enabled)
        {
        }

        break;

    default:
        break;
    }

    return current;
}
