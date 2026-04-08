/*  Channel.cpp Channel related variables and functions.
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

ChannelCategory SanitizeChannelCategory(uint8_t rawCategory)
{
    if (rawCategory >= CHANNEL_CATEGORY_COUNT)
    {
        return CHANNEL_CATEGORY_AUXILIARY;
    }

    return (ChannelCategory)rawCategory;
}

ChannelPriority GetChannelPriority(ChannelCategory category)
{
    switch (SanitizeChannelCategory((uint8_t)category))
    {
    case CHANNEL_CATEGORY_HEATED_SEATS:
    case CHANNEL_CATEGORY_HEATED_STEERING_WHEEL:
    case CHANNEL_CATEGORY_INFOTAINMENT:
    case CHANNEL_CATEGORY_USB_ACCESSORY_POWER:
    case CHANNEL_CATEGORY_DATA_LOGGER:
    case CHANNEL_CATEGORY_TELEMETRY:
    case CHANNEL_CATEGORY_CAMERA_SYSTEM:
    case CHANNEL_CATEGORY_LAP_TIMER:
    case CHANNEL_CATEGORY_COOL_SUIT_PUMP:
    case CHANNEL_CATEGORY_INTERIOR_LIGHTS:
    case CHANNEL_CATEGORY_AUXILIARY:
    case CHANNEL_CATEGORY_SPARE:
    case CHANNEL_CATEGORY_CUSTOM:
        return CHANNEL_PRIORITY_LOW;

    case CHANNEL_CATEGORY_HVAC_BLOWER:
    case CHANNEL_CATEGORY_AC_CLUTCH:
    case CHANNEL_CATEGORY_PIT_LIMITER:
        return CHANNEL_PRIORITY_MEDIUM;

    default:
        return CHANNEL_PRIORITY_CRITICAL;
    }
}

void SanitizeChannelConfig(ChannelConfig &config)
{
    if ((uint8_t)config.ChanType > (uint8_t)DIG_INTERMITTENT)
    {
        config.ChanType = DIG;
    }

    config.Category = SanitizeChannelCategory((uint8_t)config.Category);

    if (config.IntermittentOnTime > MAX_INTERMITTENT_TIME_MS)
    {
        config.IntermittentOnTime = MAX_INTERMITTENT_TIME_MS;
    }

    if (config.IntermittentOffTime > MAX_INTERMITTENT_TIME_MS)
    {
        config.IntermittentOffTime = MAX_INTERMITTENT_TIME_MS;
    }
}

