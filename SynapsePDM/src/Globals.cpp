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

ChannelConfigUnion ChannelConfigData;
AnalogueConfigUnion AnalogueConfigData;
ChannelConfig Channels[NUM_CHANNELS];
AnalogueInputs AnalogueIns[NUM_ANA_CHANNELS];

uint32_t imuWWtimer;
uint32_t DisplayTimer;
uint32_t CommsTimer;
uint32_t BattTimer;
uint32_t LogTimer;
uint32_t GPSTimer;
uint32_t BLTimer;
int blLevel = 0;

uint8_t RTCyear;
uint8_t RTCmonth;
uint8_t RTCday;
uint8_t RTChour;
uint8_t RTCminute;
uint8_t RTCsecond;

bool enabledFlags[NUM_CHANNELS] = {false};
unsigned long enabledTimers[NUM_CHANNELS] = {0};

// SPI 2
SPIClass SPI_2(PICO, POCI, SCK2);

uint8_t connectionStatus = 0;

int recBytesRead = 0;

bool backgroundDrawn = false;

void InitialiseChannelData()
{
  // Initialise channels to default values, ensure they are initially off
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    Channels[i].ChanType = DIG;
    Channels[i].Enabled = false;
    Channels[i].OutputControlPin = channelOutputPins[i];
    Channels[i].CurrentSensePin = channelCurrentSensePins[i];
    if (i < NUM_DI_CHANNELS)
    {
      Channels[i].InputControlPin = DIchannelInputPins[i];
    }
    else
    {
      Channels[i].InputControlPin = ANAchannelInputPins[i - NUM_DI_CHANNELS];
    }    
    Channels[i].CurrentThresholdHigh = CURRENT_MAX;
    Channels[i].CurrentThresholdLow = 0.0;
    pinMode(Channels[i].OutputControlPin, OUTPUT);
    digitalWrite(Channels[i].OutputControlPin, LOW);
    Channels[i].ActiveHigh = true;
    Channels[i].RunOn = false;
    Channels[i].RunOnTime = 0;
    Channels[i].MultiChannel = false;    
    Channels[i].RetryCount = 3;
    Channels[i].InrushDelay = INRUSH_DELAY;
    Channels[i].Override = false;
  } 
}

void InitialiseAnalogueData()
{
  // Initialise analogue inputs to default values
  for (int i = 0; i < NUM_ANA_CHANNELS; i++)
  {
    AnalogueIns[i].InputPin = ANAchannelInputPins[i];
    AnalogueIns[i].PullUpPin = ANAchannelInputPullUps[i];
    AnalogueIns[i].PullDownPin = ANAchannelInputPullDowns[i];
    AnalogueIns[i].PullUpEnable = false;
    AnalogueIns[i].PullDownEnable = false;
    AnalogueIns[i].IsDigital = false;
    AnalogueIns[i].IsThreshold = true;
    AnalogueIns[i].OnThreshold = 2.5;   // 2.5V on threshold
    AnalogueIns[i].OffThreshold = 2.0;  // 2.0V off threshold
    AnalogueIns[i].ScaleMin = 0.0;     // Minimum scale value
    AnalogueIns[i].ScaleMax = CURRENT_MAX; // Maximum scale value
    AnalogueIns[i].PWMMin = 0;         // Minimum PWM value
    AnalogueIns[i].PWMMax = 100;       // Maximum PWM value
  }
}
