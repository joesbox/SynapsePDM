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
    2026-01-21        v0.6          - Added watchdog timer. Different timings applied on boot and normal operation. Extended to 10 seconds during PC comms, 30 seconds during sleep.
                                    - Invalidate display flag set on sleep to force redraw on wake.
                                    - Added internal pull-up/pull-down configuration for analogue inputs to prevent false input read on wake.
                                    - Added GSM signal strength to display.
                                    - CAN messaging implementation. CAN channel configuration and EEPROM update. 5 second timeout for multiple channel config messages before saving to EEPROM.
                                    - Global RTC
                                    - Re-open last log file on wake. Maximises log file storage capacity over 10 files.
                                    - Fix: GPS status on sleep/wake.
                                    - Fix: Corrected default CAN IDs to be within standard range.
                                    - Fix: CAN bus resistor enable pin state on wake/power up.
                                    - Optimised page read and writes to EEPROM to be in page-sized chunks.
    2026-01-06        v0.5          - Added boot to sleep functionality. If enabled, system will enter deep sleep mode after initialisation until wake event.
                                    - Corrected peripheral clock enable/disable in OutputHandler sleep/wake functions. Sleep current is ~2.2mA.
                                    - Added wake debounce timer to prevent multiple wake events.
                                    - More robust sleep and wake handling.
                                    - Moved dynamic system parameters to SystemRuntime structure.
                                    - Added padding to EEPROM structures to allow for future expansion without breaking existing installations.
                                    - Added system config changes read from Cortex app.
    2025-12-16        v0.4          - Removed battery management functionality.
                                    - Removed battery measurement from display.
                                    - Removed battery measurement from logging.
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

constexpr int SPLASH_SCREEN_DELAY = 2000;
constexpr int RTC_YEAR_THRESHOLD = 24;

void SleepFunctions();
void alarmMatch(void *data);

// #define DEBUG

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
    Serial.print(ChannelRuntime[i].AnalogRaw);
    Serial.print(", ");
  }

  Serial.println();

  Serial.print("Analogue calculated current: ");
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    Serial.print(ChannelRuntime[i].CurrentValue);
    Serial.print(", ");
  }

  Serial.println();
  Serial.print("Error flags: ");*/
  Serial.print("EEPROM flags: ");
  Serial.print(SystemCRCValid ? "Valid" : "Invalid");
  Serial.print(", ");
  Serial.print(ChannelCRCValid ? "Valid" : "Invalid");
  Serial.print(", ");
  Serial.print(StorageCRCValid ? "Valid" : "Invalid");
  Serial.print(", ");
  Serial.print(AnalogueCRCValid ? "Valid" : "Invalid");
  Serial.print(", ");

  Serial.print(hitInit ? "Yep" : "Nope");
  Serial.print(", ");

  Serial.println(SystemRuntimeParams.ErrorFlags, HEX);

#endif
}

void setup()
{

  IWatchdog.begin(5000 * 1000); // 5 second watchdog (microseconds) on boot.

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
  IWatchdog.reload();

  // Load channel data first
  ChannelCRCValid = LoadChannelConfig();
  if (!ChannelCRCValid)
  {
    // CRC wasn't valid on the EEPROM channel data. Save the default values to EEPROM now.
    InitialiseChannelData();
    SaveChannelConfig();
  }

  // Load system data
  SystemCRCValid = LoadSystemConfig();
  if (!SystemCRCValid)
  {
    // CRC wasn't valid on the EEPROM system data. Save the default values to EEPROM now.
    InitialiseSystemData();
    SaveSystemConfig();
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
  InitialiseCAN();
  PowerState = RUN;

  if (IWatchdog.isReset())
  {
    IWatchdog.clearReset();

    // If we reset due to the watchdog, skip the splash screen
    splashCounter = millis();
  }
  else
  {
    // Display splash screen for set time
    splashCounter = millis() + SPLASH_SCREEN_DELAY;
  }
  digitalWrite(TFT_BL, HIGH);

  SystemParams.MotionDeadTime = 1;

  LowPower.enableWakeupFrom(&rtc, alarmMatch);
  IWatchdog.begin(2000 * 1000); // 2 second watchdog (microseconds) on boot.
}

void handlePowerState()
{
  switch (PowerState)
  {
  case RUN:
    if (!digitalRead(IGN_INPUT))
    {
      delay(WAKE_DEBOUNCE_TIME); // Debounce
      if (!digitalRead(IGN_INPUT) && bootToSleep)
      {
        PowerState = PREPARE_SLEEP;
      }
    }
    break;
  case PREPARE_SLEEP:
    if (SystemParams.AllowMotionDetect)
    {
      EnableMotionDetect();
      uint32_t ignitionOffTime = rtc.getEpoch();
      SDCardOK = false;
      HAL_PWR_EnableBkUpAccess();
      setBackupRegister(BACKUP_REG_IGN_OFF_TIME, ignitionOffTime);
    }
    SleepFunctions();
    GPSFix = false;
    PowerState = SLEEPING;
    break;
  case SLEEPING:
    if (imuWakePending)
    {
      imuWakePending = false;
      PowerState = IMU_WAKING;
    }

    IWatchdog.begin(32000 * 1000); // 32 second watchdog (microseconds) during sleep.
    IWatchdog.reload();
    rtc.setAlarmEpoch(rtc.getEpoch() + 30); // Wake every 30 seconds to feed the watchdog
    rtc.enableAlarm(rtc.MATCH_DHHMMSS);

    // Enter STOP mode
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    break;
  case IGNITION_WAKING:
    if (!IMUWakeMode)
    {
      // Only call these if we haven't woken from the IMU
      HAL_ResumeTick();
      SystemClock_Config();
      rtc.begin();
    }
    IWatchdog.reload();
    wakeDebounceTimer = millis();
    PowerState = IGNITION_WAKE;
    break;
  case IGNITION_WAKE:
    if (millis() - wakeDebounceTimer > WAKE_DEBOUNCE_TIME)
    {
      IWatchdog.begin(2000 * 1000); // 2 second watchdog (microseconds) during run.
      IWatchdog.reload();
      WakeSystem();
      InitialiseInputs();
      InitialiseOutputs();
      HandleInputs();
      UpdateOutputs();
      DisableMotionDetect();
      InitialiseCAN();
      InitialiseSerial();
      InitialiseGSM(false);
      StartDisplay();
      DrawBackground();
      analogWrite(TFT_BL, 1023);
      ResumeSD();
      PowerState = RUN;
    }
    break;
  case IMU_WAKING:
    HAL_ResumeTick();
    SystemClock_Config();
    rtc.begin();
    IWatchdog.reload();
    PowerState = IMU_WAKE;
    break;
  case IMU_WAKE:
    if (SystemParams.AllowMotionDetect)
    {
      uint32_t ignitionOffTime = getBackupRegister(BACKUP_REG_IGN_OFF_TIME);
      uint32_t currentTime = rtc.getEpoch();
      if ((currentTime - ignitionOffTime) >= (SystemParams.MotionDeadTime * 60))
      {
        // Motion dead time has elapsed. Disable motion detection, wake the system.
        IWatchdog.begin(2000 * 1000); // 2 second watchdog (microseconds) during run.
        IWatchdog.reload();
        InitialiseInputs();
        DisableMotionDetect();
        InitialiseSerial();
        WakeSystem();
        InitialiseCAN();
        InitialiseGSM(false);
        ResumeSD();
        imuWWtimer = millis() + SystemParams.IMUwakeWindow;
        PowerState = IMU_WAKE_WINDOW;
      }
      else
      {
        // Still within motion dead time. Go back to sleep.
        SleepFunctions();
        PowerState = SLEEPING;
      }
    }
    break;
  case IMU_WAKE_WINDOW:
    if (millis() < imuWWtimer)
    {
      // TODO: work out what to do if the IMU has woken the controller
    }
    else
    {
      SleepFunctions();
      PowerState = SLEEPING;
    }
    break;
  }
}

void loop()
{
  IWatchdog.reload();
  if (PowerState == RUN)
  {
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

    if (millis() > LogTimer)
    {
      LogTimer = millis() + LOG_INTERVAL;

      if (!RTCSet && year > RTC_YEAR_THRESHOLD)
      {
        RTCSet = true;        
        // GPS time must be updated, use that
        rtc.setDate(day, month, (year % 100));
        rtc.setTime(hour, minute, second);
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

    if (millis() > signalTimer)
    {
      signalTimer = millis() + SIGNAL_QUALITY_INTERVAL;
      UpdateSIM7600(SIGNAL_QUALITY);
    }

    if (millis() > systemCANTimer)
    {
      systemCANTimer = millis() + SYSTEM_CAN_INTERVAL; 
      BroadcastSystemStatus();
    }

    if (millis() > splashCounter && !backgroundDrawn && PowerState != PREPARE_SLEEP && PowerState != SLEEPING)
    {
      DrawBackground();
      bootToSleep = true;
    }
    CheckSerial();
    ReadCANMessages();
  }
  handlePowerState();
  
  if(millis() > EEPROMSaveTimout && saveEEPROMOnTimeout)
  {    
    saveEEPROMOnTimeout = false;
    EEPROMSaveTimout = 0;   
    SaveChannelConfig();    
    SaveSystemConfig();
  }
}

void SleepFunctions()
{
  if (saveEEPROMOnTimeout)
  {
    // Do it now.
    SaveChannelConfig();
    saveEEPROMOnTimeout = false;
    EEPROMSaveTimout = 0;
  }
  analogWrite(TFT_BL, 0);
  PullResistorSleep();
  SleepSD();
  OutputsOff();
  SleepOutputs();
  SleepComms();
  StopDisplay();
  SleepSystem();
  IMUWakeMode = false;
  invalidateDisplay = true;
}

void alarmMatch(void *data)
{
  // Do nothing, just wake the MCU
}