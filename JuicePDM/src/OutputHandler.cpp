/*  OutputHandler.cpp Output handler deals with channel output control.
    Specifically applies to the Infineon BTS50010 High-Side Driver
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

#include "OutputHandler.h"

volatile uint8_t analogCounter;
uint analogValues[NUM_CHANNELS][ANALOG_READ_SAMPLES];

// Channel number used to identify associated channel
int channelNum;

/// @brief Handle output control
void InitialiseOutputs()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    // Make sure all channels are off when we initialise
    pinMode(Channels[i].OutputControlPin, OUTPUT);
    digitalWrite(Channels[i].OutputControlPin, LOW);
  }

  // Reset the counters
  analogCounter = 0;
}

/// @brief Update PWM or digital outputs
void UpdateOutputs()
{
  // Check the type of channel we're dealing with (digital or PWM) and handle output accordingly
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    switch (Channels[i].ChanType)
    {
    case DIG:
    case CAN_DIGITAL:
      if (Channels[i].Enabled)
      {

        digitalWriteFast(digitalPinToPinName(Channels[i].OutputControlPin), HIGH);
        int sum = 0;
        uint8_t total = 0;
        for (int j = 0; j < ANALOG_READ_SAMPLES; j++)
        {
          sum += analogRead(Channels[i].CurrentSensePin);
          total++;
        }
        float analogMean = 0.0f;
        if (total)
        {
          analogMean = sum / total;
          Channels[i].AnalogRaw = analogMean;
        }

        // TODO: Calibrate current readings for the BTS50025

        float isVoltage = (Channels[i].AnalogRaw / 1024.0) * 3.3;
        float amps = PTERM1 * pow(isVoltage, 4) + PTERM2 * pow(isVoltage, 3) + PTERM3 * pow(isVoltage, 2) + PTERM4 * isVoltage + PCONST;

        // Check for fault condition and current thresholds
        if (isVoltage > FAULT_THRESHOLD)
        {
          Channels[i].ErrorFlags |= IS_FAULT;
        }
        else if (amps > Channels[i].CurrentThresholdHigh)
        {
          Channels[i].ErrorFlags |= CHN_OVERCURRENT_RANGE;
        }
        else if (amps < Channels[i].CurrentThresholdLow)
        {
          Channels[i].ErrorFlags |= CHN_UNDERCURRENT_RANGE;
        }
        else if (amps > Channels[i].CurrentLimitHigh)
        {
          Channels[i].ErrorFlags |= CHN_OVERCURRENT_LIMIT;
        }
        else
        {
          // No conditions found. Clear flag
          Channels[i].ErrorFlags = 0;
        }

        Channels[i].CurrentValue = amps;
      }
      else
      {
        digitalWriteFast(digitalPinToPinName(Channels[i].OutputControlPin), LOW);
        Channels[i].CurrentValue = 0.0;
      }
      break;
    default:
      digitalWriteFast(digitalPinToPinName(Channels[i].OutputControlPin), LOW);
      Channels[i].CurrentValue = 0.0;
      break;
    }
  }
}

void OutputsOff()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    digitalWriteFast(digitalPinToPinName(Channels[i].OutputControlPin), LOW);
    Channels[i].CurrentValue = 0.0;
    Channels[i].ErrorFlags = 0;
  }
}