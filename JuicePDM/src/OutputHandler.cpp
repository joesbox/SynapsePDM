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

volatile bool channelOutputStatus[NUM_CHANNELS];

volatile uint32_t analogReadIntervals[NUM_CHANNELS];

PeriodicTimer analogPWMReadTImer;

PeriodicTimer analogDigitalReadTImer;

PeriodicTimer calculateAnalogsTimer;

CRGB leds[NUM_CHANNELS];

uint8_t toggle[NUM_CHANNELS];

/// @brief Handle output control
void HandleOutputs()
{
  // Configure output pin interrupts on the rising edge for PWM outputs
  attachInterrupt(Channels[0].ControlPin, CH1_ISR, RISING);
  attachInterrupt(Channels[1].ControlPin, CH2_ISR, RISING);
  attachInterrupt(Channels[2].ControlPin, CH3_ISR, RISING);
  attachInterrupt(Channels[3].ControlPin, CH4_ISR, RISING);
  attachInterrupt(Channels[4].ControlPin, CH5_ISR, RISING);
  attachInterrupt(Channels[5].ControlPin, CH6_ISR, RISING);

  // Analog PWM read interval timer start
  analogPWMReadTImer.begin(ReadPWMAnalogs, ANALOG_PWM_READ_INTERVAL, true);

  // Analog digital read interval timer start
  analogDigitalReadTImer.begin(ReadDigitalAnalogs, ANALOG_DIGITAL_READ_INTERVAL, true);

  // Start PWM
  SoftPWMBegin();
}

/// @brief Update PWM or digital outputs
void UpdateOutputs()
{
  // Check the type of channel we're dealing with (digital or PWM) and handle output accordingly
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    switch (Channels[i].ChanType)
    {
    case DIG_ACT_LOW_PWM:
    case DIG_ACT_HIGH_PWM:
    case CAN_PWM:
      if (Channels[i].Enabled)
      {
        float squared = (VBATT_NOMINAL / SystemParams.VBatt) * (VBATT_NOMINAL / SystemParams.VBatt);
        uint8_t pwmActual = round(Channels[i].PWMSetDuty * squared);
        SoftPWMSet(Channels[i].ControlPin, pwmActual);
        if (Channels[i].ErrorFlags == 0)
        {
          leds[i] = CRGB::DeepSkyBlue;
        }
        else
        {
          if (toggle[i] >= 128)
          {
              leds[i] = CRGB::DeepSkyBlue;
              if (toggle[i] == 255)
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
        }
      }
      else
      {
        SoftPWMSet(Channels[i].ControlPin, 0);
        leds[i] = CRGB::Black;
      }
      break;
    case DIG_ACT_LOW:
    case DIG_ACT_HIGH:
    case CAN_DIGITAL:
      if (Channels[i].Enabled)
      {
        digitalWrite(Channels[i].ControlPin, HIGH);
        analogReadIntervals[i] = ARM_DWT_CYCCNT;
        leds[i] = CRGB::DarkGreen;
      }
      else
      {
        digitalWrite(Channels[i].ControlPin, LOW);
        leds[i] = CRGB::Black;
      }
      break;
    default:
      digitalWrite(Channels[i].ControlPin, LOW);
      leds[i] = CRGB::Black;
      break;
    }
  }
  FastLED.show();
}

/// @brief Take analog readings at the pre-defined interval for PWM-enabled channels
void ReadPWMAnalogs()
{
#ifdef DEBUG
  digitalWrite(ANALOG_READ_DEBUG_PIN, HIGH); // Useful for scoping outputs to validate timing of analog reads
#endif

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    if (channelOutputStatus[i])
    {
      // Ensure the maximum turn on time for the BTS50010 has passed before taking an analog reading. Reset the output status flag.
      if (ARM_DWT_CYCCNT - analogReadIntervals[i] >= ANALOG_DELAY / CPU_TICK_MICROS)
      {
        Channels[i].AnalogRaw = analogRead(Channels[i].CurrentSensePin);
        channelOutputStatus[i] = false;
      }
    }
  }
#ifdef DEBUG
  digitalWrite(ANALOG_READ_DEBUG_PIN, LOW);
#endif
}

/// @brief Take analog readings for digital-enabled channels
void ReadDigitalAnalogs()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    switch (Channels[i].ChanType)
    {
    case DIG_ACT_LOW:
    case DIG_ACT_HIGH:
    case CAN_DIGITAL:
      // Ensure the maximum turn on time for the BTS50010 has passed before taking an analog reading.
      if (ARM_DWT_CYCCNT - analogReadIntervals[i] >= ANALOG_DELAY / CPU_TICK_MICROS)
      {
        Channels[i].AnalogRaw = analogRead(Channels[i].CurrentSensePin);
      }
      break;
    default:
      break;
    }
  }
}

/// @brief Calculate real current value in amps
void CalculateAnalogs()
{
  // TODO: implement current reading formula. Will need to consider calibration points.
}

/// @brief Channel 1 ISR. Sets the channel output status to true and sets the clock cycle counter value
void CH1_ISR()
{
  channelOutputStatus[0] = true;
  analogReadIntervals[0] = ARM_DWT_CYCCNT;
}

/// @brief Channel 2 ISR. Sets the channel output status to true and sets the clock cycle counter value
void CH2_ISR()
{
  channelOutputStatus[1] = true;
  analogReadIntervals[1] = ARM_DWT_CYCCNT;
}

/// @brief Channel 3 ISR. Sets the channel output status to true and sets the clock cycle counter value
void CH3_ISR()
{
  channelOutputStatus[2] = true;
  analogReadIntervals[2] = ARM_DWT_CYCCNT;
}

/// @brief Channel 4 ISR. Sets the channel output status to true and sets the clock cycle counter value
void CH4_ISR()
{
  channelOutputStatus[3] = true;
  analogReadIntervals[3] = ARM_DWT_CYCCNT;
}

/// @brief Channel 5 ISR. Sets the channel output status to true and sets the clock cycle counter value
void CH5_ISR()
{
  channelOutputStatus[4] = true;
  analogReadIntervals[4] = ARM_DWT_CYCCNT;
}

/// @brief Channel 6 ISR. Sets the channel output status to true and sets the clock cycle counter value
void CH6_ISR()
{
  channelOutputStatus[5] = true;
  analogReadIntervals[5] = ARM_DWT_CYCCNT;
}

void InitialiseLEDs()
{
  FastLED.addLeds<WS2812B, RGB_PIN, GRB>(leds, NUM_CHANNELS);
  uint8_t brightness = 0;
  bool latch = false;

  for (int j = 0; j < 256; j++)
  {
    if (brightness < 128 && !latch)
    {
      FastLED.setBrightness(brightness++);
    }
    else
    {
      FastLED.setBrightness(brightness--);
      latch = true;
    }
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      leds[i] = Scroll((i * 256 / NUM_CHANNELS + j) % 256);
    }

    FastLED.show();
    delay(5);
  }
  FastLED.showColor(CRGB::Black);
  FastLED.setBrightness(SystemParams.LEDBrightness);
}

CRGB Scroll(int pos)
{
  CRGB color(0, 0, 0);
  if (pos < 85)
  {
    color.g = 0;
    color.r = ((float)pos / 85.0f) * 255.0f;
    color.b = 255 - color.r;
  }
  else if (pos < 170)
  {
    color.g = ((float)(pos - 85) / 85.0f) * 255.0f;
    color.r = 255 - color.g;
    color.b = 0;
  }
  else if (pos < 256)
  {
    color.b = ((float)(pos - 170) / 85.0f) * 255.0f;
    color.g = 255 - color.b;
    color.r = 1;
  }
  return color;
}
