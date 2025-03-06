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
#include <GSM.h>
#include <Display.h>

void wdtCallback();

STM32RTC &rtc1 = STM32RTC::getInstance();


void setup()
{
  rtc1.setClockSource(STM32RTC::LSE_CLOCK);
  rtc1.begin();
  RTCSet = false;

  InitialiseSerial();
  InitialiseSystem();

  // Only initialise the SD card if we've got an accurate RTC
  if (rtc1.isTimeSet() && rtc1.getYear() > 24)
  {
    InitialiseSD();
  }

  InitialiseOutputs();
  InitialiseInputs();

  // Load system data
  SystemCRCValid = LoadSystemConfig();
  if (!SystemCRCValid)
  {
    // CRC wasn't valid on the EEPROM system data. Save the default vales to EEPROM now.
    InitialiseSystemData();
    SaveSystemConfig();
  }

  // Load channel data
  ChannelCRCValid = LoadChannelConfig();
  if (!ChannelCRCValid)
  {
    // CRC wasn't valid on the EEPROM system data. Save the default vales to EEPROM now.
    InitialiseChannelData();
    SaveChannelConfig();
  }

  // Load storage data
  StorageCRCValid = LoadStorageConfig();
  if (!StorageCRCValid)
  {
    // CRC wasn't valid on the EEPROM system data. Save the default vales to EEPROM now.
    InitialiseStorageData();
    SaveStorageConfig();
  }

  InitialiseIMU();
  InitialiseGSM(false);
  // InitialiseDisplay();

  LogTimer = millis() + (1.0 / (float)StorageParams.LogFrequency * 1000.0); // Hz to milliseconds
  PowerState = RUN;

#ifdef DEBUG
  Serial.println("Power up");
#endif
  task0Timer = task1Timer = task2Timer = task3Timer = task4Timer = debugTimer = imuWWtimer = 0;
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

      // Read input channel status
      HandleInputs();
    }

    // Lower priority tasks
    if (millis() >= task2Timer)
    {
      task2Timer = millis() + TASK_2_INTERVAL;
      // Update system parameters
      UpdateSystem();

      // Broadcast CAN updates
      // SendCANMessages();
    }

    // Lower priority tasks
    if (millis() >= task3Timer)
    {
      task3Timer = millis() + TASK_3_INTERVAL;

      // Check for serial comms
      CheckSerial();
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

    // GPS
    if (millis() >= GPStimer)
    {
      GPStimer = millis() + GPS_INTERVAL;
      UpdateGPS();
    }

    // Logging
    if (millis() >= LogTimer)
    {
      LogTimer = millis() + (1.0 / (float)StorageParams.LogFrequency * 1000.0); // Hz to milliseconds
      if (!RTCSet)
      {
        if (year > 2024)
        {
          RTCSet = true;
          // GPS time must be updated, use that
          rtc1.setDate(day, month, (year % 100));
          rtc1.setTime(hour, minute, second);
          InitialiseSD();
        }
      }
      else
      {
        // RTC is set. Log SD card data
        LogData();
      }
    }

    // Debug
    if (millis() >= debugTimer)
    {
      debugTimer = millis() + DEBUG_INTERVAL;
#ifdef DEBUG
      Serial.print("System temperature: ");
      Serial.println(SystemParams.SystemTemperature);
      Serial.print("System error flags: ");
      Serial.println(SystemParams.ErrorFlags);
      Serial.print("System voltage: ");
      Serial.println(SystemParams.VBatt);
      Serial.print("System date: ");
      Serial.print(rtc1.getYear());
      Serial.print("-");
      Serial.print(rtc1.getMonth());
      Serial.print("-");
      Serial.println(rtc1.getDay());
      Serial.print("System time: ");
      Serial.print(rtc1.getHours());
      Serial.print(":");
      Serial.print(rtc1.getMinutes());
      Serial.print(":");
      Serial.println(rtc1.getSeconds());
      Serial.print("System CRC Valid: ");
      Serial.println(SystemCRCValid);
      Serial.print("Channel CRC Valid: ");
      Serial.println(ChannelCRCValid);
      Serial.print("Storage CRC Valid: ");
      Serial.println(StorageCRCValid);
      Serial.print("Power state: ");
      Serial.println(PowerState);
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
      Serial.print("Latitude: ");
      Serial.println(lat, 6);
      Serial.print("Longitude: ");
      Serial.println(lon, 6);
      Serial.print("Speed: ");
      Serial.println(speed);
      Serial.print("Altitude: ");
      Serial.println(alt);
      Serial.print("Visible satellites: ");
      Serial.println(vsat);
      Serial.print("Used satellites: ");
      Serial.println(usat);
      Serial.print("Accuracy: ");
      Serial.println(accuracy);
      Serial.print("Year: ");
      Serial.println(year);
      Serial.print("Month: ");
      Serial.println(month);
      Serial.print("Day: ");
      Serial.println(day);
      Serial.print("Hour: ");
      Serial.println(hour);
      Serial.print("Minute: ");
      Serial.println(minute);
      Serial.print("Second: ");
      Serial.println(second);
#endif
    }
    break;
  case PREPARE_SLEEP:
#ifdef DEBUG
    Serial.println("Prepare sleep");
#endif
    // TODO: Implement channel run-on timers
    EnableMotionDetect();
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
    // InitialiseSD();
    DisableMotionDetect();
    InitialiseSerial();
    WakeSystem();
    PowerState = RUN;
    break;

  case IMU_WAKE:
    DisableMotionDetect();
    WakeSystem();
    imuWWtimer = millis() + SystemParams.IMUwakeWindow;
    PowerState = IMU_WAKE_WINDOW;
    break;
  case IMU_WAKE_WINDOW:
    // While we're in the wake window
    if (millis() < imuWWtimer)
    {
      // TODO: work out what to do if the IMU has woken the controller
    }
    else
    {
      // Nothing happening, go back to sleep
      PowerState = PREPARE_SLEEP;
    }
    break;
  }
}