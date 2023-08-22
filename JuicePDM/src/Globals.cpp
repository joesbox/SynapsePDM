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
elapsedMillis task1;
elapsedMillis task2;
elapsedMillis task3;
ChannelConfig Channels[NUM_CHANNELS];

/// @brief Inititlise global data
void InititalizeData()
{
    // Initialise channels to default values, ensure they are initially off
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        Channels[i].ChanType = DIG_ACT_HIGH;
        Channels[i].Enabled = false;
        Channels[i].ControlPin = channelOutputPins[i];
        Channels[i].CurrentSensePin = channelCurrentSensePins[i];
        Channels[i].CurrentSenseValue = DEFAULT_DK_VALUE;
        Channels[i].InputControlPin = channelInputPins[i];
        pinMode(Channels[i].InputControlPin, INPUT);
        pinMode(Channels[i].ControlPin, OUTPUT);
        digitalWrite(Channels[i].ControlPin, LOW);
    }

    // Initialise default system data
    SystemParams.LEDBrightness = DEFAULT_RGB_BRIGHTNESS;

    // Sync time with PC if available
    while (!Serial && millis() < 4000)
        ;
    Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
    setSyncProvider(getTeensy3Time);
}

// Get current time
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}