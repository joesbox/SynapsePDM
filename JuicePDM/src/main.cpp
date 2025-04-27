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
    2025-03-09        v0.1          Initial beta release.
*/

#include <SystemClock.h>
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
#include <Battery.h>

STM32RTC &rtc = STM32RTC::getInstance();

constexpr int SPLASH_SCREEN_DELAY = 2000;
constexpr int RTC_YEAR_THRESHOLD = 24;

void Debug()
{
#ifdef DEBUG
  /* Serial.print("System temperature: ");
   Serial.println(SystemParams.SystemTemperature);
   Serial.print("System error flags: ");
   Serial.println(SystemParams.ErrorFlags);
   Serial.print("System voltage: ");
   Serial.println(SystemParams.VBatt);
   Serial.print("Log interval: ");
   Serial.println(LogTask.getInterval());
   Serial.print("RTC Set: ");
   Serial.println(RTCSet);
   Serial.print("System date: ");
   Serial.print(rtc.getYear());
   Serial.print("-");
   Serial.print(rtc.getMonth());
   Serial.print("-");
   Serial.println(rtc.getDay());
   Serial.print("System time: ");
   Serial.print(rtc.getHours());
   Serial.print(":");
   Serial.print(rtc.getMinutes());
   Serial.print(":");
   Serial.println(rtc.getSeconds());
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

  const float imuData[] = {accelX, accelY, accelZ, gyroX, gyroY, gyroZ};
  const char *imuLabels[] = {"Accel X", "Accel Y", "Accel Z", "Rotation X", "Rotation Y", "Rotation Z"};
  for (int i = 0; i < 6; ++i)
  {
    Serial.print(imuLabels[i]);
    Serial.print(": ");
    Serial.println(imuData[i], 3);
  }

  /*Serial.print("Latitude: ");
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
  //printBatteryStats();*/

  const float imuData[] = {accelX, accelY, accelZ, gyroX, gyroY, gyroZ};
  const char *imuLabels[] = {"Accel X", "Accel Y", "Accel Z", "Rotation X", "Rotation Y", "Rotation Z"};
  for (int i = 0; i < 6; ++i)
  {
    Serial.print(imuLabels[i]);
    Serial.print(": ");
    Serial.println(imuData[i], 3);
  }

  Serial.print("Log file bytes stored: ");
  Serial.println(BytesStored);
  Serial.print("Log lines, freq:");
  Serial.print(StorageParams.MaxLogLength);
  Serial.print(", ");
  Serial.println(StorageParams.LogFrequency);
  Serial.print("Battery SOC: ");
  Serial.println(SOC);
  Serial.print("Analogue raw current: ");
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    Serial.print(Channels[i].AnalogRaw);
    Serial.print(", ");
  }

  Serial.println();

  Serial.print("Analogue calculated current: ");
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    Serial.print(Channels[i].CurrentValue);
    Serial.print(", ");
  }

  Serial.println();

  Serial.println(SystemParams.ErrorFlags, HEX);

#endif
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  InitialiseSystem();
  InitialiseGSM(false);
  analogWrite(TFT_BL, 0);
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin();
  InitialiseSerial();
  InitialiseOutputs();
  InitialiseInputs();
  InitialiseStorageData();
  InitialiseDisplay();
  InitialiseChannelData();
  InitialiseBattery();

  // Load system data
  SystemCRCValid = LoadSystemConfig();
  if (!SystemCRCValid)
  {
    // CRC wasn't valid on the EEPROM system data. Save the default vales to EEPROM now.
    InitialiseSystemData();
    SaveSystemConfig();
  }

  // Load channel data
  ChannelCRCValid = false; // LoadChannelConfig();
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

  // Only initialise the SD card if we've got an accurate RTC
  if (rtc.isTimeSet() && rtc.getYear() > 24)
  {
    InitialiseSD();
    RTCSet = true;
  }

  InitialiseIMU();
  PowerState = RUN;

  // Display the splash screen for 2 seconds
  splashCounter = millis() + SPLASH_SCREEN_DELAY;
}

void handlePowerState()
{
  switch (PowerState)
  {
  case RUN:
    if (!digitalRead(IGN_INPUT))
    {
      PowerState = PREPARE_SLEEP;
    }
    break;
  case PREPARE_SLEEP:
#ifdef DEBUG
    Serial.println("Prepare sleep");
#endif
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
    DisableMotionDetect();
    InitialiseSerial();
    WakeSystem();
    InitialiseDisplay();
    PowerState = RUN;
    break;
  case IMU_WAKE:
    DisableMotionDetect();
    WakeSystem();
    imuWWtimer = millis() + SystemParams.IMUwakeWindow;
    PowerState = IMU_WAKE_WINDOW;
    break;
  case IMU_WAKE_WINDOW:
    if (millis() < imuWWtimer)
    {
      // TODO: work out what to do if the IMU has woken the controller
    }
    else
    {
      PowerState = PREPARE_SLEEP;
    }
    break;
  }
}

uint32_t start;

void loop()
{

  if (millis() > DisplayTimer)
  {
    DisplayTimer = millis() + DISPLAY_INTERVAL;
    // Update channel outputs
    UpdateOutputs();

    // Read input channel status
    HandleInputs();
    if (backgroundDrawn)
    {
      UpdateDisplay();
    }
    UpdateSystem();
  }

  if (millis() > CommsTimer)
  {
    CommsTimer = millis() + COMMS_INTERVAL;
    CheckSerial();
    ReadIMU();
  }

  if (millis() > BattTimer)
  {
    BattTimer = millis() + BATTERY_INTERVAL;
    ManageBattery();
  }

  if (millis() > LogTimer)
  {
    LogTimer = millis() + LOG_INTERVAL;
    RTCyear = rtc.getYear();
    RTCmonth = rtc.getMonth();
    RTCday = rtc.getDay();
    RTChour = rtc.getHours();
    RTCminute = rtc.getMinutes();
    RTCsecond = rtc.getSeconds();

    if (!RTCSet && year > RTC_YEAR_THRESHOLD)
    {
      RTCSet = true;
      // GPS time must be updated, use that
      rtc.setDate(day, month, (year % 100));
      rtc.setTime(hour, minute, second);
      RTCyear = rtc.getYear();
      RTCmonth = rtc.getMonth();
      RTCday = rtc.getDay();
      RTChour = rtc.getHours();
      RTCminute = rtc.getMinutes();
      RTCsecond = rtc.getSeconds();
      InitialiseSD();
    }
    else if (RTCSet)
    {
      // RTC is set. log SD card data
      LogData();
    }
  }

  if (millis() > GPSTimer)
  {
    GPSTimer = millis() + GPS_INTERVAL;
    start = millis();
    UpdateSIM7600(GPS);
    Serial.print("GPS time: ");
    Serial.println(millis() - start);
    start = millis();
    Debug();
    Serial.print("Debug time: ");
    Serial.println(millis() - start);
  }

  if (millis() > splashCounter && !backgroundDrawn)
  {
    DrawBackground();
  }
  handlePowerState();
}