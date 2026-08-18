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
    2026-06-10        v0.10         - Wake sequence and display optimisations.
                                    - Added configurable CAN baud rates.
                                    - Extended CAN config protocol to include more parameters (DBC updated and wiki markdown added to this repository).
                                    - Telemetry implementation using OpenRemote. See https://wiki.joeblogs.uk for more info.
    2026-04-22        v0.9          - Wake/sleep bug fix.
                                    - Increase system temperature limit to 100°C. Max. STM32 operating junction temp is 105°C.
                                    - Further sleep current improvements down to ~1mA.
                                    - Assert output on wake bug fix.
                                    - Undercurrent lock out fix.
                                    - Added new CAN messages for I/O status.
                                    - Inrush current added to CAN configuration.
                                    - Calibration added to serial protocol for automated testing purposes.
                                    - Outputs now store per-channel current sense calibration values.
                                    - Save changes while outputs active bug fix.
    2026-03-17        v0.8          - Added manual RTC update command over Cortex serial protocol for installations not using GPS.
                                    - Run-on implementation
                                    - Soft start & soft stop implementation
                                    - Re-factor analogue input parameters. Now stored on a per-channel basis rather than in the analogue input structure. Allows for more flexible configuration of analogue inputs.
                                    - Analogue threshold and PWM scaled outputs now implemented.
                                    - Analogue inputs can be configured for different sensor types and units. Display and logging updated to show correct units.
                                    - Added new parameters to CAN messages.
                                    - PWM frequency changed to 150Hz to balance switching losses and heat dissipation.
                                    - Added all inputs to logs.
                                    - Minor EEPROM tweaks.
                                    - Task scheduling re-factored to be more efficient and robust. Tasks now run on a best effort basis at their specified intervals rather than being strictly scheduled.
                                    - Application now has two build options. Default is with the SD card bootloader.
                                    - Start of firmware update implementation. Currently supports receiving firmware update packets over serial and writing to flash.
                                    - Serial protocol refactor.
                                    - Addition of BMM350 for supported boards.
                                    - Temperature from SIM and IMU added to system parameters and logging.
                                    - GPS plausibility filter. Helps stop outliers being logged due to bad GPS fixes.
                                    - Channel categorisation and prioritisation for staged thermal shutdown.
                                    - New channel type: intermittent.
                                    - New input option: on with ignition/wake.
                                    - Daylight saving time support for RTC.
                                    - GPS wake AT command bug fix.
                                    - Minor sleep/wake fixes and optimisations.
    2026-02-18        v0.7          - Fixed display config. Disabled warnings about (non-existent) touch screen.
                                    - Minor display tweaks.
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

static void ResetLogScheduler()
{
  uint32_t nowMs = millis();
  LogTimer = micros() + LOG_INTERVAL_US;
  GPSTimer = nowMs + GPS_INTERVAL;
  signalTimer = nowMs + SIGNAL_QUALITY_INTERVAL;
  simTemperatureTimer = nowMs + SIM_TEMPERATURE_INTERVAL;
}

static bool ignitionDelayedOffStateCaptured = false;

static void CaptureIgnitionDelayedOffState(uint32_t now)
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    bool ignitionTriggeredDelayedOff = Channels[i].DelayedOff &&
                                       Channels[i].DelayedOffTrigger == DELAYED_OFF_IGNITION_OFF;
    ignitionDelayedOffEligible[i] = ignitionTriggeredDelayedOff && IsChannelRuntimeEnabled(i);
    ignitionDelayedOffDeadline[i] = ignitionDelayedOffEligible[i] ? (now + Channels[i].DelayedOffTime) : 0;
  }

  ignitionDelayedOffStateCaptured = true;
}

static void ResetIgnitionDelayedOffState()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    ignitionDelayedOffEligible[i] = false;
    ignitionDelayedOffDeadline[i] = 0;
  }

  ignitionDelayedOffStateCaptured = false;
}

static void ClearWakeRequests()
{
  ignitionWakePending = false;
  imuWakePending = false;
}

void SleepFunctions()
{
  if (saveEEPROMOnTimeout)
  {
    PersistPendingCANConfigChanges();
    saveEEPROMOnTimeout = false;
    EEPROMSaveTimout = 0;
  }
  ResetChannelInputTimingStates();
  ResetGPSPlausibilityFilter();
  analogWrite(TFT_BL, 0);
  PullResistorSleep();
  SleepIMU(SystemParams.AllowMotionDetect != 0);
  SleepSD();
  SleepCAN();
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
  // Stop the alarm after it fires so it cannot keep retriggering later sleep entries.
  rtc.disableAlarm();
}

void setup()
{
  IWatchdog.begin(5000 * 1000); // 5 second watchdog (microseconds) on boot.

  InitialiseSystem();
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);
  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin();
  InitialiseSerial();
  InitialiseOutputs();
  InitialiseStorageData();
  InitialiseDisplay();
  InitialiseChannelData();

  if (FORCE_EEPROM_WIPE_ON_BOOT)
  {
    CleanEEPROM();

    InitialiseChannelData();
    SaveChannelConfig();

    InitialiseSystemData();
    SaveSystemConfig();

    InitialiseStorageData();
    SaveStorageConfig();

    InitialiseAnalogueData();
    SaveAnalogueConfig();

    InitialiseCellularData();
    SaveCellularConfig();
  }

  IWatchdog.reload();

  // Load channel data first
  ChannelCRCValid = LoadChannelConfig();
  if (!ChannelCRCValid)
  {
    // CRC wasn't valid on the EEPROM channel data. Save the default values to EEPROM now.
    InitialiseChannelData();
    SaveChannelConfig();
    ChannelCRCValid = LoadChannelConfig();
  }

  // Load system data
  SystemCRCValid = LoadSystemConfig();
  if (!SystemCRCValid)
  {
    // CRC wasn't valid on EEPROM system data. Save defaults once and re-check.
    InitialiseSystemData();
    SaveSystemConfig();
    SystemCRCValid = LoadSystemConfig();
  }

  // Load storage data
  StorageCRCValid = LoadStorageConfig();
  if (!StorageCRCValid)
  {
    // CRC wasn't valid on the EEPROM system data. Save the default vales to EEPROM now.
    InitialiseStorageData();
    SaveStorageConfig();
    StorageCRCValid = LoadStorageConfig();
  }

  // Load analogue input data
  AnalogueCRCValid = LoadAnalogueConfig();
  if (!AnalogueCRCValid)
  {
    // CRC wasn't valid on the EEPROM system data. Save the default vales to EEPROM now.
    InitialiseAnalogueData();
    SaveAnalogueConfig();
    AnalogueCRCValid = LoadAnalogueConfig();
  }

  // Load cellular/OpenRemote data
  CellularCRCValid = LoadCellularConfig();
  if (!CellularCRCValid)
  {
    InitialiseCellularData();
    SaveCellularConfig();
    CellularCRCValid = LoadCellularConfig();
  }

  if (ChannelCRCValid && ChannelConfigNeedsRewriteAfterLoad)
  {
    SaveChannelConfig();
  }

  if (SystemCRCValid && SystemConfigNeedsRewriteAfterLoad)
  {
    SaveSystemConfig();
  }

  InitialiseInputs();

  // Only initialise the SD card if we've got an accurate RTC
  if (HasUsableRtcTime())
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

  LowPower.enableWakeupFrom(&rtc, alarmMatch);
  ResetLogScheduler();
  IWatchdog.begin(2000 * 1000); // 2 second watchdog (microseconds) on boot.
  InitialiseGSM();
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
        CaptureIgnitionDelayedOffState(millis());
        PowerState = PREPARE_SLEEP;
      }
    }
    else if (ignitionDelayedOffStateCaptured)
    {
      ResetIgnitionDelayedOffState();
    }
    break;
  case PREPARE_SLEEP:
  {
    // Delay sleep if any channel is still within its ignition-off delayed-off window.
    bool ignitionDelayedOffActive = false;
    uint32_t now = millis();

    if (digitalRead(IGN_INPUT))
    {
      ResetIgnitionDelayedOffState();
      analogWrite(TFT_BL, 1023);
      invalidateDisplay = true;
      PowerState = RUN;
      break;
    }

    // Fallback capture in case PREPARE_SLEEP is entered from a path other than RUN.
    if (!ignitionDelayedOffStateCaptured)
    {
      CaptureIgnitionDelayedOffState(now);
    }
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      bool ignitionTriggeredDelayedOff = Channels[i].DelayedOff &&
                                         Channels[i].DelayedOffTrigger == DELAYED_OFF_IGNITION_OFF;
      if (ignitionTriggeredDelayedOff && ignitionDelayedOffEligible[i])
      {
        analogWrite(TFT_BL, 0);

        if (ignitionDelayedOffDeadline[i] != 0 && (int32_t)(now - ignitionDelayedOffDeadline[i]) < 0)
        {
          ignitionDelayedOffActive = true;
        }
        else
        {
          ignitionDelayedOffEligible[i] = false;
          ignitionDelayedOffDeadline[i] = 0;
        }
      }
      else
      {
        ignitionDelayedOffDeadline[i] = 0;
      }
    }
    if (ignitionDelayedOffActive)
    {
      // Wait until all ignition-off delayed-off channels have expired before sleeping.
      IWatchdog.reload();
      HandleInputs();
      UpdateOutputs();
      break;
    }
    else
    {
      ResetIgnitionDelayedOffState();
    }

    if (digitalRead(IGN_INPUT))
    {
      ResetIgnitionDelayedOffState();
      analogWrite(TFT_BL, 1023);
      invalidateDisplay = true;
      PowerState = RUN;
      break;
    }

    // The sleep teardown path can take longer than the normal 2 second run watchdog.
    IWatchdog.begin(32000 * 1000);
    IWatchdog.reload();

    // Only call sleep functions after all ignition-off delayed-off channels have finished.
    if (SystemParams.AllowMotionDetect)
    {
      EnableMotionDetect();
      uint32_t ignitionOffTime = rtc.getEpoch();
      SDCardOK = false;
      HAL_PWR_EnableBkUpAccess();
      setBackupRegister(BACKUP_REG_IGN_OFF_TIME, ignitionOffTime);
    }
    ClearWakeRequests();
    SleepFunctions();
    GPSFix = false;
    PowerState = SLEEPING;
  }
  break;
  case SLEEPING:
    // Treat ignition-high as a pending wake even if the rising edge arrived during sleep teardown.
    if (digitalRead(IGN_INPUT) || ignitionWakePending)
    {
      ClearWakeRequests();
      IMUWakeMode = false;
      PowerState = IGNITION_WAKING;
      break;
    }

    if (imuWakePending)
    {
      imuWakePending = false;
      PowerState = IMU_WAKING;
    }

    if (PowerState != SLEEPING)
    {
      break;
    }

    IWatchdog.begin(32000 * 1000); // 32 second watchdog (microseconds) during sleep.
    IWatchdog.reload();
    rtc.disableAlarm();
    rtc.setAlarmEpoch(rtc.getEpoch() + 30); // Wake every 30 seconds to feed the watchdog
    rtc.enableAlarm(rtc.MATCH_DHHMMSS);

    if (PowerState != SLEEPING)
    {
      break;
    }

    if (digitalRead(IGN_INPUT) || ignitionWakePending)
    {
      ClearWakeRequests();
      IMUWakeMode = false;
      PowerState = IGNITION_WAKING;
      break;
    }

    if (imuWakePending)
    {
      imuWakePending = false;
      PowerState = IMU_WAKING;
      break;
    }

    // Enter STOP mode
    HAL_SuspendTick();
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    SystemClock_Config();
    HAL_ResumeTick();

    if (digitalRead(IGN_INPUT) || ignitionWakePending)
    {
      ClearWakeRequests();
      IMUWakeMode = false;
      PowerState = IGNITION_WAKING;
      break;
    }

    if (imuWakePending)
    {
      imuWakePending = false;
      PowerState = IMU_WAKING;
    }

    break;
  case IGNITION_WAKING:
    rtc.disableAlarm();
    ClearWakeRequests();
    WakeSource = WAKE_SOURCE_IGNITION;
    IWatchdog.reload();
    wakeDebounceTimer = millis();
    PowerState = IGNITION_WAKE;
    break;
  case IGNITION_WAKE:
    if (millis() - wakeDebounceTimer > WAKE_DEBOUNCE_TIME)
    {
      // pinMode(IGN_INPUT, INPUT_PULLDOWN);
      if (!digitalRead(IGN_INPUT))
      {
        PowerState = SLEEPING;
        break;
      }

      IWatchdog.begin(5000 * 1000); // Allow extra time for wake reinitialisation before returning to the run watchdog.
      IWatchdog.reload();
      WakeSystem();
      IWatchdog.reload();
      InitialiseInputs();
      InitialiseOutputs();
      HandleInputs();
      UpdateOutputs();
      IWatchdog.reload();
      ReinitialiseIMUAfterWake();
      DisableMotionDetect();
      InitialiseCAN();
      InitialiseSerial();
      InitialiseGSM();
      // Check inputs and outputs again as an external ECU may have asserted I/O on power up.
      HandleInputs();
      UpdateOutputs();
      IWatchdog.reload();
      StartDisplay();
      DrawBackground();
      analogWrite(TFT_BL, 1023);
      IWatchdog.reload();
      ResumeSD();
      ResetLogScheduler();
      IWatchdog.begin(2000 * 1000); // Restore the normal run watchdog once wake initialisation is complete.
      IWatchdog.reload();
      // Reset ignition-off delayed-off eligibility and timers on wake
      for (int i = 0; i < NUM_CHANNELS; i++)
      {
        enabledTimers[i] = 0;
      }
      ResetIgnitionDelayedOffState();
      ClearWakeRequests();
      IMUWakeMode = false;

      PowerState = RUN;
    }
    break;
  case IMU_WAKING:
    rtc.disableAlarm();
    ignitionWakePending = false;
    WakeSource = WAKE_SOURCE_IMU;
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
        IWatchdog.begin(5000 * 1000); // Allow extra time for wake reinitialisation before returning to the run watchdog.
        IWatchdog.reload();
        WakeSystem();
        pinMode(TFT_RST, OUTPUT);
        digitalWrite(TFT_RST, LOW);
        analogWrite(TFT_BL, 0);
        IWatchdog.reload();
        ReinitialiseIMUAfterWake();
        InitialiseInputs();
        DisableMotionDetect();
        InitialiseSerial();
        InitialiseCAN();
        InitialiseGSM();
        IWatchdog.reload();
        ResumeSD();
        ResetLogScheduler();
        IWatchdog.begin(2000 * 1000); // Restore the normal run watchdog once wake initialisation is complete.
        IWatchdog.reload();
        ClearWakeRequests();
        IMUWakeMode = false;
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
    if ((int32_t)(millis() - imuWWtimer) < 0)
    {
      // TODO: work out what to do if the IMU has woken the controller
    }
    else
    {
      if (SystemParams.AllowMotionDetect)
      {
        EnableMotionDetect();
      }

      SleepFunctions();
      PowerState = SLEEPING;
    }
    break;
  }
}

void loop()
{
  IWatchdog.reload();
  uint8_t powerStateBefore = PowerState;
  handlePowerState();

  if (powerStateBefore == RUN && PowerState == RUN)
  {
    CheckSerial();
    bool logTransferActive = IsLogTransferActive();
    bool cortexSaveActive = IsCortexConfigSaveActive();
    uint32_t now = millis();
    uint32_t logNow = micros();

    // Skip non-critical tasks during Cortex config saves to prioritize serial comms
    if (!cortexSaveActive)
    {
      if ((int32_t)(now - DisplayTimer) >= 0)
      {
        DisplayTimer += DISPLAY_INTERVAL;
        if ((int32_t)(now - DisplayTimer) >= 0)
        {
          DisplayTimer = now + DISPLAY_INTERVAL;
        }

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

      if (!logTransferActive && (int32_t)(now - CommsTimer) >= 0)
      {
        CommsTimer += COMMS_INTERVAL;
        if ((int32_t)(now - CommsTimer) >= 0)
        {
          CommsTimer = now + COMMS_INTERVAL;
        }

        ReadIMU();
      }

      if ((int32_t)(logNow - LogTimer) >= 0)
      {
        LogTimer += LOG_INTERVAL_US;
        if ((int32_t)(logNow - LogTimer) >= 0)
        {
          LogTimer = logNow + LOG_INTERVAL_US;
        }

        if (GPSFix)
        {
          ApplyUtcRtcDateTime((uint16_t)year, (uint8_t)month, (uint8_t)day, (uint8_t)hour, (uint8_t)minute, (uint8_t)second);
        }

        if (RTCSet)
        {
          // RTC is set. log SD card data
          LogData();
        }
      }

      if (!logTransferActive && (int32_t)(now - GPSTimer) >= 0)
      {
        GPSTimer += GPS_INTERVAL;
        if ((int32_t)(now - GPSTimer) >= 0)
        {
          GPSTimer = now + GPS_INTERVAL;
        }

        UpdateSIM7600(GPS);
      }

      if (!logTransferActive && (int32_t)(now - signalTimer) >= 0)
      {
        signalTimer += SIGNAL_QUALITY_INTERVAL;
        if ((int32_t)(now - signalTimer) >= 0)
        {
          signalTimer = now + SIGNAL_QUALITY_INTERVAL;
        }

        UpdateSIM7600(SIGNAL_QUALITY);
      }

      if (!logTransferActive && (int32_t)(now - simTemperatureTimer) >= 0)
      {
        simTemperatureTimer += SIM_TEMPERATURE_INTERVAL;
        if ((int32_t)(now - simTemperatureTimer) >= 0)
        {
          simTemperatureTimer = now + SIM_TEMPERATURE_INTERVAL;
        }

        UpdateSIM7600(MODULE_TEMPERATURE);
      }

      if (!logTransferActive)
      {
        UpdateSIM7600();
      }

      if ((int32_t)(now - systemCANTimer) >= 0)
      {
        systemCANTimer += SYSTEM_CAN_INTERVAL;
        if ((int32_t)(now - systemCANTimer) >= 0)
        {
          systemCANTimer = now + SYSTEM_CAN_INTERVAL;
        }

        BroadcastSystemStatus();
      }

      if (!logTransferActive && (int32_t)(now - splashCounter) >= 0 && !backgroundDrawn && PowerState != PREPARE_SLEEP && PowerState != SLEEPING)
      {
        DrawBackground();
        bootToSleep = true;
      }
    }

    if (logTransferActive)
    {
      CheckSerial();
    }

    // Skip CAN reading during config saves to avoid SPI contention
    if (!cortexSaveActive)
    {
      ReadCANMessages();
    }
  }

  if (saveEEPROMOnTimeout && (int32_t)(millis() - EEPROMSaveTimout) >= 0)
  {
    saveEEPROMOnTimeout = false;
    EEPROMSaveTimout = 0;
    PersistPendingCANConfigChanges();
  }
}
