/*  OutputHandler.cpp Output handler deals with channel output control.
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
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE,
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
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
    digitalWrite(Channels[i].ControlPin, LOW);

    channelLatch[i] = 0;
  }

  // Reset the counters
  pwmCounter = analogCounter = 0;

  /*adc->adc0->setConversionSpeed(ADC_settings::ADC_CONVERSION_SPEED::LOW_SPEED);
  adc->adc1->setConversionSpeed(ADC_settings::ADC_CONVERSION_SPEED::LOW_SPEED);

  adc->adc0->setSamplingSpeed(ADC_settings::ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc1->setSamplingSpeed(ADC_settings::ADC_SAMPLING_SPEED::LOW_SPEED);

  adc->adc0->setAveraging(32);
  adc->adc1->setAveraging(32);*/

  pinMode(20, OUTPUT);
  // Start PWM interval timer
 // myTimer.begin(OutputTimer, PWM_COUNT_INTERVAL);
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

        /*if (Channels[i].ErrorFlags == 0)
        {
          leds[i] = CRGB::DeepSkyBlue;
        }
        else
        {
          if (toggle[i] >= 10)
          {
            leds[i] = CRGB::DeepSkyBlue;
            if (toggle[i] == 20)
            {
              toggle[i] = 0;
            }
            toggle[i]++;
          }
          else
          {
            leds[i] = CRGB::DarkRed;
            toggle[i]++;
          }
        }*/

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
        digitalWrite(Channels[i].ControlPin, HIGH);

        // We've written the channel high once, no need to keep writing the channel high until we've reached the end of the Ton period
        channelLatch[i] = 1;
      }
      else if (realPWMValues[i] < pwmCounter && channelLatch[i])
      {
        // We are within th Toff period of the duty cycle, keep the channel off
        digitalWrite(Channels[i].ControlPin, LOW);

        // No need to keep writing the pin low either
        channelLatch[i] = 0;
      }
    }
    else
    {
      digitalWrite(Channels[i].ControlPin, LOW);
    }
  }

  // Increment the PWM counter at every interval
  pwmCounter++;
}