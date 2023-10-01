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

CRGB leds[NUM_CHANNELS];

uint8_t toggle[NUM_CHANNELS];

/// @brief Handle output control
void InitialiseOutputs()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    pinMode(Channels[i].CurrentSensePin, INPUT_DISABLE);
  }
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
    case DIG_PWM:
    case CAN_PWM:
      if (Channels[i].Enabled)
      {
        float squared = (VBATT_NOMINAL / SystemParams.VBatt) * (VBATT_NOMINAL / SystemParams.VBatt);
        int pwmActual = round(Channels[i].PWMSetDuty * squared);

        if (pwmActual > 255)
        {
          pwmActual = 255;
        }
        else if (pwmActual < 0)
        {
          pwmActual = 0;
        }
        SoftPWMSet(Channels[i].ControlPin, Channels[i].CurrentSensePin, Channels[i].CurrentSensePWM, pwmActual);
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

        // Read the analog raw back
        Channels[i].AnalogRaw = SoftPWMGetAnalog(Channels[i].ControlPin);
      }
      else
      {
        SoftPWMSet(Channels[i].ControlPin, Channels[i].CurrentSensePin, Channels[i].CurrentSensePWM, 0);
        leds[i] = CRGB::Black;
      }
      break;
    case DIG:
    case CAN_DIGITAL:
      if (Channels[i].Enabled)
      {
        // Use SoftPWM library so we still get analog sampling
        SoftPWMSet(Channels[i].ControlPin, Channels[i].CurrentSensePin, Channels[i].CurrentSensePWM, 255);
        leds[i] = CRGB::DarkGreen;
      }
      else
      {
        SoftPWMSet(Channels[i].ControlPin, Channels[i].CurrentSensePin, Channels[i].CurrentSensePWM, 0);
        leds[i] = CRGB::Black;
      }
      break;
    default:
      SoftPWMSet(Channels[i].ControlPin, Channels[i].CurrentSensePin, Channels[i].CurrentSensePWM, 0);
      leds[i] = CRGB::Black;
      break;
    }
  }
  FastLED.show();
}

/// @brief Calculate real current value in amps
void CalculateAnalogs()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    
  }
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
