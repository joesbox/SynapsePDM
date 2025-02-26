/*  SynapsePDM - CAN enabled Power Distribution Module with 14 channels.

    Code herein specifically applies to the application of Infineon BTS50025 High-Side Drivers
    on the SynapsePDM hardware. See https://wiki.joeblogs.uk for more info.

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

  InitialiseIMU();

#ifdef DEBUG
  Serial.println("Power up");
#endif
  task0Timer = task1Timer = task2Timer = task3Timer = task4Timer = 0;
}

void loop()
{
  switch (PowerState)
  {
  case RUN:
    // Check ignition input
    if (!digitalRead(IGN_INPUT))
    {
      PowerState = PREPARE_SLEEP;
    }

    if (millis() >= task0Timer)
    {
      task0Timer = millis() + TASK_0_INTERVAL;
      ReadIMU();
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
      Serial.print("IMU OK: ");
      Serial.println(IMUOK);
      Serial.print("IMU data (Accel X, Y Z, Rotation X, Y Z): ");
      Serial.print(accelX, 3);
      Serial.print(", ");
      Serial.print(accelY, 3);
      Serial.print(", ");
      Serial.print(accelZ, 3);
      Serial.print(", ");
      Serial.print(gyroX, 3);
      Serial.print(", ");
      Serial.print(gyroY, 3);
      Serial.print(", ");
      Serial.println(gyroZ, 3);
#endif
    }
    break;
  case PREPARE_SLEEP:
    // TODO: Implement channel run-on timers
    PullResistorSleep();
    SleepSD();
    OutputsOff();
    SleepComms();
    SleepSystem();
    PowerState = SLEEPING;
    break;

  case SLEEPING:

    LowPower.deepSleep();
    break;

  case IGNITION_WAKE:
    InitialiseSD();
    InitialiseSerial();
    WakeSystem();
    PowerState = RUN;
    break;

  case IMU_WAKE:
    WakeSystem();  
    PowerState = RUN;
    break;

  default:
    break;
  }
}