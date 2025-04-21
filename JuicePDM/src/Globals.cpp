/*  Globals.h Global variables, definitions and functions.
    Copyright (c) 2023 Joe Mann.  All right reserved.

    This work is licensed under the Creative Commons
    Attribution-NonCommercial-ShareAlike 4.0 International License.
    To view a copy of this license, visit
    https://creativecommons.org/licenses/by-nc-sa/4.0/ or send a
    letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

    You are free to:
    - Share: Copy and redistribute the material in any medium or format.
    - Adapt: Remix, transform, and build upon the material.

    Under the following terms:
    - Attribution: You must give appropriate credit, provide a link to the license,
      and indicate if changes were made. You may do so in any reasonable manner,
      but not in any way that suggests the licensor endorses you or your use.
    - NonCommercial: You may not use the material for commercial purposes.
    - ShareAlike: If you remix, transform, or build upon the material,
      you must distribute your contributions under the same license as the original.

    DISCLAIMER: This software is provided "as is," without warranty of any kind,
    express or implied, including but not limited to the warranties of
    merchantability, fitness for a particular purpose, and noninfringement.
    In no event shall the authors or copyright holders be liable for any claim,
    damages, or other liability, whether in an action of contract, tort, or otherwise,
    arising from, out of, or in connection with the software or the use or
    other dealings in the software.
*/

#include "Globals.h"

ChannelConfigUnion ChannelConfigData;
ChannelConfig Channels[NUM_CHANNELS];
AnalogueInputs AnalogueIns[NUM_ANA_CHANNELS];

uint32_t imuWWtimer;
uint32_t DisplayTimer;
uint32_t CommsTimer;
uint32_t BattTimer;
uint32_t LogTimer;
uint32_t GPSTimer;

uint8_t RTCyear;
uint8_t RTCmonth;
uint8_t RTCday;
uint8_t RTChour;
uint8_t RTCminute;
uint8_t RTCsecond;

// SPI 2
SPIClass SPI_2(PICO, POCI, SCK2);

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
}
