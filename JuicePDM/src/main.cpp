/*  JuicePDM - CAN enabled Power Distribution Module with 6 channels.

    Code herein specifically applies to the application of Infineon BTS50010 High-Side Drivers
    on the JuicePDM hardware. See https://wiki.joeblogs.uk for more info.

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

    Version history:
    2023-08-14        v0.1.0        Initial beta
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Bounce2.h>
#include <Globals.h>
#include <OutputHandler.h>
#include <InputHandler.h>
#include <Storage.h>
#include <Watchdog_t4.h>
#include <CANComms.h>

WDT_T4<WDT1> wdt;

void wdtCallback(); 

void setup()
{
  Serial.begin(9600);

  WDT_timings_t config;
  config.trigger = 3; /* in seconds, 0->128 */
  config.timeout = 5; /* in seconds, 0->128 */  
  config.callback = wdtCallback;
  wdt.begin(config);

  InititalizeData();

  // LED debugging
  /*
  Channels[0].ChanType = DIG_ACT_HIGH;
  Channels[0].Enabled = true;

  Channels[1].ChanType = CAN_PWM;
  Channels[1].Enabled = true;
  Channels[1].PWMSetDuty = 10;

  Channels[2].ChanType = DIG_ACT_HIGH;
  Channels[2].Enabled = true;

  Channels[3].ChanType = DIG_ACT_HIGH;
  Channels[3].Enabled = true;

  Channels[4].ChanType = DIG_ACT_HIGH;
  Channels[4].Enabled = true;

  Channels[5].ChanType = DIG_ACT_HIGH;
  Channels[5].Enabled = true;  
  */

  InitialiseLEDs();
  HandleOutputs();
  CRCFailed = LoadConfig();
}

void loop()
{
  // High priority tasks
  if (task1 >= TASK_1_INTERVAL)
  {
    // Update channel outputs
    UpdateOutputs();

    // Read input channel status
    HandleInputs();

    task1 = 0;
  }

  // Lower priority tasks
  if (task2 >= TASK_2_INTERVAL)
  {
    // Update system parameters
    //UpdateSystem();

    // Broadcast CAN updates
    //SendCANMessages();

    task2 = 0;
  }

  // Lower priority tasks
  if (task3 >= TASK_3_INTERVAL)
  {
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      Serial.print("Pin: ");
      Serial.println(Channels[i].CurrentSensePin);
      Serial.print("Raw value: ");
      Serial.println(Channels[i].AnalogRaw);
    }
    task3 = 0;
  }

  // Feed the dog.
  wdt.feed();
}

void wdtCallback()
{
    // If we've landed here, the watchdog looks like it may timeout. Set the error flag which may or may not get logged to the SD card.
    SystemParams.ErrorFlags |= WATCHDOG_TIMEOUT;
}