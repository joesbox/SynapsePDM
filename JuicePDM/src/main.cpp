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
    2024-12-09        v0.1.1        New hardware. Moving to the STM32 for increased I/O. Now 14 output channels.
    2023-08-14        v0.1.0        Initial beta
*/

// #include <SystemClock.h>
#include <Arduino.h>
#include <Bounce2.h>
#include <Globals.h>
#include <OutputHandler.h>
#include <InputHandler.h>
#include <Storage.h>
#include <CANComms.h>
#include <SerialComms.h>

void wdtCallback();

String lastThingCalled;

STM32RTC &rtc1 = STM32RTC::getInstance();

void setup()
{

  InitialiseSerial();
  InititalizeData();

  InitialiseSD();

  InitialiseOutputs();
  InitialiseInputs();
  CRCValid = LoadConfig();
  InitialiseSystem();
  rtc1.begin();

#ifdef DEBUG
  Serial.println("Power up");
  AnalogueIns[0].PullUpEnable = true;
  digitalWrite(PWR_EN_5V, HIGH);
  digitalWrite(PWR_EN_3V3, HIGH);
#endif
  task1Timer = task2Timer = task3Timer = task4Timer = 0;
}

void loop()
{
  switch (PowerState)
  {
  case RUN:
    // Check ignition input
    if (!digitalRead(IGN_INPUT))
    {
      // PowerState = PREPARE_SLEEP;
    }

    // High priority tasks
    if (millis() >= task1Timer)
    {
      task1Timer = millis() + TASK_1_INTERVAL;
      // Update channel outputs
      UpdateOutputs();

      lastThingCalled = "UpdateOutputs";

      // Read input channel status
      HandleInputs();

      lastThingCalled = "HandleInputs";
    }

    // Lower priority tasks
    if (millis() >= task2Timer)
    {
      task2Timer = millis() + TASK_2_INTERVAL;
      // Update system parameters
      UpdateSystem();
      lastThingCalled = "UpdateSystem";

      // Broadcast CAN updates
      // SendCANMessages();
    }

    // Lower priority tasks
    if (millis() >= task3Timer)
    {
      task3Timer = millis() + TASK_3_INTERVAL;
      // Log SD card data
      LogData();

      // Check for serial comms
      CheckSerial();

      lastThingCalled = "LogData";
    }

    // Lowest priority tasks
    if (millis() >= task4Timer)
    {
      task4Timer = millis() + TASK_4_INTERVAL;
      char temp[6];
      sprintf(temp, "%02d", rtc1.getHours());
      Serial.print(temp);
      Serial.print(":");
      sprintf(temp, "%02d", rtc1.getMinutes());
      Serial.print(temp);
      Serial.print(":");
      sprintf(temp, "%02d", rtc1.getSeconds());
      Serial.print(temp);
      Serial.print(" - ");
      Serial.println("One minute has passed...");
    }

    // Debug
    if (millis() >= debugTimer)
    {
      debugTimer = millis() + DEBUG_INTERVAL;
#ifdef DEBUG
      Serial.print("System temperature: ");
      Serial.println(SystemParams.SystemTemperature);
      Serial.print("CRC Valid: ");
      Serial.println(CRCValid);
      Serial.print("System error flags: ");
      Serial.println(SystemParams.ErrorFlags, HEX);
#endif
    }
    break;
  case PREPARE_SLEEP:

    // TODO: Implement channel run-on timers
    break;

  case SLEEPING:
    LowPower.deepSleep();
    break;

  case IGNITION_WAKE:

    break;

  case IMU_WAKE:
    break;

  default:
    break;
  }
}