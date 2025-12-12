/*  SynapsePDM - CAN enabled Power Distribution Module with 14 channels.

    Code herein specifically applies to the application of Infineon BTS50025 High-Side Drivers
    on the SynapsePDM hardware. See https://wiki.joeblogs.uk for more info.

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

    ! UPDATE FW VERSION DEFINED IN GLOBALS.H !

    Version history:
    Date              Version       Description
    ----              -------       ------------------------------------------------------------
    2025-12-11        v0.3          Fixes:
                                    - Buffered serial writes.    
                                    - Analogue input data stored to EEPROM.
                                    - Update analogue input parameters from Cortex.                                
                                    - Check input type and pin when evaluating digital inputs.
    2025-12-02        v0.2          Fixes:
                                    - Added clearing of channel error flags before storage.
                                    - Clear channel error flags on disable.
                                    - Ensure channel name is null-terminated before display.
                                    - Overrides cleared on serial timeout and save to EEPROM.
                                    - Decreased serial timeout to 5 seconds.
                                    - Removed display backlight fade-in logic to prevent wake issues.
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

  /*const float imuData[] = {accelX, accelY, accelZ, gyroX, gyroY, gyroZ};
  const char *imuLabels[] = {"Accel X", "Accel Y", "Accel Z", "Rotation X", "Rotation Y", "Rotation Z"};
  for (int i = 0; i < 6; ++i)
  {
    Serial.print(imuLabels[i]);
    Serial.print(": ");
    Serial.println(imuData[i], 3);
  }*/

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
  Serial.print("Error flags: ");

  Serial.println(SystemParams.ErrorFlags, HEX);

#endif
}

void setup()
{
  InitialiseSystem();
  InitialiseGSM(false);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);  
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin();
  InitialiseSerial();
  InitialiseOutputs();  
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

  // Load analogue input data
  AnalogueCRCValid = LoadAnalogueConfig();
  if (!AnalogueCRCValid)
  {
    // CRC wasn't valid on the EEPROM system data. Save the default vales to EEPROM now.
    InitialiseAnalogueData();
    SaveAnalogueConfig();
  }

  InitialiseInputs();

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
  digitalWrite(TFT_BL, HIGH);
}

void handlePowerState()
{
  switch (PowerState)
  {
  case RUN:
    if (!digitalRead(IGN_INPUT))
    {
      delay(100); // Debounce
      if (!digitalRead(IGN_INPUT))
      {
        PowerState = PREPARE_SLEEP;
      }
    }
    break;
  case PREPARE_SLEEP:
    EnableMotionDetect();
    PullResistorSleep();
    SleepSD();
    OutputsOff();
    SleepOutputs();
    SleepComms();
    StopDisplay();
    SleepSystem();
    PowerState = SLEEPING;
    delay(100);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    break;
  case SLEEPING:

    break;
  case IGNITION_WAKE:
    DisableMotionDetect();
    InitialiseSerial();
    WakeSystem();
    analogWrite(TFT_BL, 1023);
    StartDisplay();
    DrawBackground();
    PowerState = RUN;
    break;
  case IMU_WAKE:
    SystemClock_Config();
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

void loop()
{
  /*if (!DisplayBacklightInitialised)
  {
    if (millis() > BLTimer)
    {
      BLTimer = millis() + BL_FADE_INTRVAL;
      if (blLevel <= 1023)
      {
        analogWrite(TFT_BL, blLevel++);
        if (blLevel > 1023)
        {
          blLevel = 1023;
          DisplayBacklightInitialised = true;
        }
      }
    }
  }
  else
  {
    analogWrite(TFT_BL, 1023);
  }*/
  
  if (millis() > DisplayTimer)
  {
    DisplayTimer = millis() + DISPLAY_INTERVAL;
    // Update channel outputs
    UpdateOutputs();

    // Read input channel status
    HandleInputs();

    // If we're heading for sleep, don't update the display. Something with the DMA seems to keeep the SPI bus active. Drastically increases sleep current.
    if (backgroundDrawn && PowerState != PREPARE_SLEEP && PowerState != SLEEPING)
    {
      UpdateDisplay();
    }
    UpdateSystem();
  }

  if (millis() > CommsTimer)
  {
    CommsTimer = millis() + COMMS_INTERVAL;
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
    UpdateSIM7600(GPS);
    Debug();
  }

  if (millis() > splashCounter && !backgroundDrawn && PowerState != PREPARE_SLEEP && PowerState != SLEEPING)
  {
    DrawBackground();
  }
  handlePowerState();
  CheckSerial();
}