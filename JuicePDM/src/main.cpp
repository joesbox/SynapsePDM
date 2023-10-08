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

void wdtCallback();

String lastThingCalled;

void setup()
{

  while (!Serial)
  {
  }
  InititalizeData();

  InitialiseSD();

  InitialiseLEDs();
  InitialiseOutputs();
  CRCValid = LoadConfig();

  Serial.println("Power up");

  // LED debugging

  Channels[0].ChanType = DIG_PWM;
  Channels[0].Enabled = false;
  Channels[0].PWMSetDuty = 25;

  Channels[1].ChanType = DIG_PWM;
  Channels[1].Enabled = false;
  Channels[1].PWMSetDuty = 25;

  Channels[2].ChanType = DIG_PWM;
  Channels[2].Enabled = false;
  Channels[2].PWMSetDuty = 25;

  Channels[3].ChanType = DIG_PWM;
  Channels[3].Enabled = false;
  Channels[3].PWMSetDuty = 25;

  Channels[4].ChanType = DIG_PWM;
  Channels[4].Enabled = false;
  Channels[4].PWMSetDuty = 25;

  Channels[5].ChanType = DIG_PWM;
  Channels[5].Enabled = true;
  Channels[5].PWMSetDuty = 25;
  task1 = task2 = task3 = task4 = 0;
}

void loop()
{
  // High priority tasks
  if (task1 >= TASK_1_INTERVAL)
  {
    // Update channel outputs
    UpdateOutputs();

    lastThingCalled = "UpdateOutputs";

    // Read input channel status
    HandleInputs();

    lastThingCalled = "HandleInputs";
    task1 = 0;
  }

  // Lower priority tasks
  if (task2 >= TASK_2_INTERVAL)
  {
    // Update system parameters
    UpdateSystem();
    lastThingCalled = "UpdateSystem";

    // Broadcast CAN updates
    // SendCANMessages();

    task2 = 0;
  }

  // Lower priority tasks
  if (task3 >= TASK_3_INTERVAL)
  {
    // Log SD card data
    LogData();
    lastThingCalled = "LogData";

    task3 = 0;
  }

  // Lowest priority tasks
  if (task4 >= TASK_4_INTERVAL)
  {
    task4 = 0;
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");
    Serial.print(second());
    Serial.print(" - ");
    Serial.println("One minute has passed...");
  }
}