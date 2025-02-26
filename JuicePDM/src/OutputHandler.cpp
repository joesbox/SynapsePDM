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

//IntervalTimer myTimer;

volatile bool channelOutputStatus[NUM_CHANNELS];

volatile uint32_t analogReadIntervals[NUM_CHANNELS];

volatile uint8_t pwmCounter;
volatile uint8_t analogCounter;
uint analogValues[NUM_CHANNELS][ANALOG_READ_SAMPLES];
volatile uint8_t realPWMValues[NUM_CHANNELS];
volatile uint8_t channelLatch[NUM_CHANNELS];


uint8_t toggle[NUM_CHANNELS];


/// @brief Handle output control
void InitialiseOutputs()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    // Disable anything related to digital input on the analogue inputs
    //pinMode(Channels[i].CurrentSensePin, INPUT_DISABLE);

    // Make sure all channels are off when we initialise
    digitalWrite(Channels[i].OutputControlPin, LOW);

    channelLatch[i] = 0;
  }

  // Reset the counters
  pwmCounter = analogCounter = 0;

  // Power rail enable outputs
  pinMode(PWR_EN_5V, OUTPUT);
  pinMode(PWR_EN_3V3, OUTPUT);
}

/// @brief Update PWM or digital outputs
void UpdateOutputs()
{
  // Check the type of channel we're dealing with (digital or PWM) and handle output accordingly
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    switch (Channels[i].ChanType)
    {
    case DIG_PWM:
    case CAN_PWM:
      if (Channels[i].Enabled)
      {
        // Calculate the adjusted PWM for volage/average power
        // float squared = (VBATT_NOMINAL / SystemParams.VBatt) * (VBATT_NOMINAL / SystemParams.VBatt);
        // int pwmActual = round(Channels[i].PWMSetDuty * squared);

        int pwmActual = Channels[i].PWMSetDuty;

        if (pwmActual > 255)
        {
          pwmActual = 255;
        }
        else if (pwmActual < 0)
        {
          pwmActual = 0;
        }
        realPWMValues[i] = pwmActual;

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

        float isVoltage = (Channels[i].AnalogRaw / 1024.0) * 3.3;
        float amps = PTERM1 * pow(isVoltage, 4) + PTERM2 * pow(isVoltage, 3) + PTERM3 * pow(isVoltage, 2) + PTERM4 * isVoltage + PCONST;
#ifdef DEBUG
        Serial.print("Channel: ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(amps);
#endif
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

        Serial.println(Channels[i].ErrorFlags, BIN);
      }
      else
      {
        realPWMValues[i] = 0;
        //leds[i] = CRGB::Black;
      }
      break;
    case DIG:
    case CAN_DIGITAL:
      if (Channels[i].Enabled)
      {
        // Digital channels get 100% duty
        realPWMValues[i] = 255;
        //leds[i] = CRGB::DarkGreen;

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
      }
      else
      {
        realPWMValues[i] = 0;
      }
      break;
    default:
      realPWMValues[i] = 0;
      break;
    }
  }
}

/// @brief This is called at an interval of every PWM_COUNT_INTERVAL. All channels use the same timing but will be enabled or disabled based on their duty
void OutputTimer()
{
  // Check the type of channel we're dealing with (digital or PWM) and handle output accordingly
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    if (Channels[i].Enabled)
    {
      if (realPWMValues[i] >= pwmCounter && !channelLatch[i])
      {
        // We are within the Ton perdiod of the duty cycle, keep the channel on
        digitalWrite(Channels[i].OutputControlPin, HIGH);

        // We've written the channel high once, no need to keep writing the channel high until we've reached the end of the Ton period
        channelLatch[i] = 1;
      }
      else if (realPWMValues[i] < pwmCounter && channelLatch[i])
      {
        // We are within th Toff period of the duty cycle, keep the channel off
        digitalWrite(Channels[i].OutputControlPin, LOW);

        // No need to keep writing the pin low either
        channelLatch[i] = 0;
      }
    }
    else
    {
      digitalWrite(Channels[i].OutputControlPin, LOW);
    }
  }

  // Increment the PWM counter at every interval
  pwmCounter++;
}