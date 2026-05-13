
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
ChannelConfigRuntime ChannelRuntime[NUM_CHANNELS];
AnalogueInputs AnalogueIns[NUM_ANA_CHANNELS];

uint32_t imuWWtimer;
uint32_t DisplayTimer;
uint32_t CommsTimer;
uint32_t LogTimer;
uint32_t GPSTimer;
uint32_t signalTimer;
uint32_t simTemperatureTimer;
uint32_t BLTimer;
uint32_t wakeDebounceTimer;
uint32_t systemCANTimer;
int blLevel = 0;

STM32RTC &rtc = STM32RTC::getInstance();

bool enabledFlags[NUM_CHANNELS] = {false};
unsigned long enabledTimers[NUM_CHANNELS] = {0};

// SPI 2
SPIClass SPI_2(PICO, POCI, SCK2);

uint8_t connectionStatus = 0;

bool pcCommsOK = false;

int recBytesRead = 0;

bool backgroundDrawn = false;

bool bootToSleep = false;

volatile bool IMUWakeMode = false;

volatile bool ignitionWakePending = false;

volatile bool imuWakePending = false;

bool CANChannelEnableFlags[NUM_CHANNELS] = {false};

// Tracks which channels are eligible for RunOn (were enabled at PREPARE_SLEEP entry)
bool runOnEligible[NUM_CHANNELS] = {false};
uint32_t runOnDeadline[NUM_CHANNELS] = {0};

void InitialiseChannelData()
{
  // Initialise channels to default values, ensure they are initially off
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    memset(&Channels[i], 0, sizeof(ChannelConfig));
    Channels[i].ChanType = DIG;
    Channels[i].Category = CHANNEL_CATEGORY_AUXILIARY;
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
    Channels[i].CurrentThresholdHigh = 1.0;
    Channels[i].CurrentThresholdLow = 0.0;
    Channels[i].OnThreshold = 2.5;
    Channels[i].OffThreshold = 2.0;
    Channels[i].ScaleMin = 0.0;
    Channels[i].ScaleMax = 5.0;
    Channels[i].PWMMin = 0;
    Channels[i].PWMMax = 100;
    pinMode(Channels[i].OutputControlPin, OUTPUT);
    digitalWrite(Channels[i].OutputControlPin, LOW);
    Channels[i].ActiveHigh = true;
    Channels[i].RunOn = false;
    Channels[i].RunOnTime = 0;
    Channels[i].MultiChannel = false;    
    Channels[i].RetryCount = 3;
    Channels[i].InrushDelay = INRUSH_DELAY;
    Channels[i].SoftStart = false;
    Channels[i].SoftStartTime = 0;
    Channels[i].SoftStop = false;
    Channels[i].SoftStopTime = 0;
    Channels[i].InrushCurrentThreshold = 1.0;
    Channels[i].IntermittentOnTime = 1000;
    Channels[i].IntermittentOffTime = 1000;
    Channels[i].CurrentSenseKILIS = DEFAULT_CHANNEL_CURRENT_SENSE_KILIS;
    ChannelRuntime[i].Override = false;
    ChannelRuntime[i].Enabled = false;
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
    AnalogueIns[i].ChanType = RAW_VOLTAGE;
    AnalogueIns[i].PullUpEnable = false;
    AnalogueIns[i].PullDownEnable = false;
    AnalogueIns[i].InputVoltage = 0.0f;
    AnalogueIns[i].InputValue = 0.0f;
    AnalogueIns[i].Units = ANA_UNITS_VOLTS;
    AnalogueIns[i].CalibrationPoints = 2;
    AnalogueIns[i].CalibrationVolt1 = 0.0f;
    AnalogueIns[i].CalibrationValue1 = 0.0f;
    AnalogueIns[i].CalibrationVolt2 = 5.0f;
    AnalogueIns[i].CalibrationValue2 = 5.0f;
    AnalogueIns[i].CalibrationVolt3 = 5.0f;
    AnalogueIns[i].CalibrationValue3 = 5.0f;
    AnalogueIns[i].NTCBeta = 3950.0f;
    AnalogueIns[i].NTCNominalResistance = 10000.0f;
  }
}
