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
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "OutputHandler.h"

volatile bool channelOutputStatus[NUM_CHANNELS];

volatile uint32_t analogReadIntervals[NUM_CHANNELS];

IntervalTimer analogReadTImer;

void Run()
{
  // Configure output pin interrupts on the rising edge for PWM outputs
  attachInterrupt(Channels[0].ControlPin, CH1_ISR, RISING);
  attachInterrupt(Channels[1].ControlPin, CH2_ISR, RISING);
  attachInterrupt(Channels[2].ControlPin, CH3_ISR, RISING);
  attachInterrupt(Channels[3].ControlPin, CH4_ISR, RISING);
  attachInterrupt(Channels[4].ControlPin, CH5_ISR, RISING);
  attachInterrupt(Channels[5].ControlPin, CH6_ISR, RISING);

  // Analog read interval timer start
  analogReadTImer.begin(ReadAnalogs, ANALOG_READ_INTERVAL);

  // Sets the interrupt priority for the analog read timer
  analogReadTImer.priority(ANALOG_READ_TIMER_PRIORITY);

  // Start PWM
  SoftPWMBegin();

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
        SoftPWMSet(Channels[i].ControlPin, Channels[i].PWMSetDuty);
      }
      else
      {
        SoftPWMSet(Channels[i].ControlPin, 0);
      }
      break;
    case DIG_ACT_LOW:
    case DIG_ACT_HIGH:
    case CAN_DIGITAL:
      if (Channels[i].Enabled)
      {
        digitalWrite(Channels[i].ControlPin, HIGH);
      }
      else
      {
        digitalWrite(Channels[i].ControlPin, LOW);
      }
      break;
    default:
      digitalWrite(Channels[i].ControlPin, LOW);
      break;
    }
  }
}

/// @brief Take analog readings at the pre-defined interval
void ReadAnalogs()
{  
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    if (channelOutputStatus[i])
    {
      if (ARM_DWT_CYCCNT - analogReadIntervals[i] >= ANALOG_DELAY / CPU_TICK_MICROS)
      {
        // TODO: implement analog read
      }
    }
  }

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
