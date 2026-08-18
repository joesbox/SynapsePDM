/*  System.cpp System variables, functions and system wide data handling.
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

#include "System.h"

#include "GSM.h"

SystemConfigUnion SystemConfigData;
SystemParameters SystemParams;
SystemRuntime SystemRuntimeParams;
volatile ThermalProtectionStage ActiveThermalProtectionStage = THERMAL_PROTECTION_NONE;

bool SystemCRCValid;
bool ChannelCRCValid;
bool SDCardOK;
volatile uint8_t PowerState;
bool RTCSet;
bool DisplayBacklightInitialised = false;

static bool IsLeapYear(uint16_t fullYear)
{
    if ((fullYear % 4U) != 0U)
    {
        return false;
    }

    if ((fullYear % 100U) != 0U)
    {
        return true;
    }

    return (fullYear % 400U) == 0U;
}

static uint8_t GetDaysInMonth(uint16_t fullYear, uint8_t month)
{
    switch (month)
    {
    case 1:
    case 3:
    case 5:
    case 7:
    case 8:
    case 10:
    case 12:
        return 31;

    case 4:
    case 6:
    case 9:
    case 11:
        return 30;

    case 2:
        return IsLeapYear(fullYear) ? 29 : 28;

    default:
        return 31;
    }
}

static int32_t DaysFromCivil(int32_t year, uint32_t month, uint32_t day)
{
    year -= month <= 2U;
    const int32_t era = (year >= 0 ? year : year - 399) / 400;
    const uint32_t yearOfEra = (uint32_t)(year - era * 400);
    const uint32_t monthPrime = month + (month > 2U ? (uint32_t)-3 : 9U);
    const uint32_t dayOfYear = (153U * monthPrime + 2U) / 5U + day - 1U;
    const uint32_t dayOfEra = yearOfEra * 365U + yearOfEra / 4U - yearOfEra / 100U + dayOfYear;
    return era * 146097 + (int32_t)dayOfEra - 719468;
}

static void CivilFromDays(int32_t days, uint16_t *fullYear, uint8_t *month, uint8_t *day)
{
    days += 719468;
    const int32_t era = (days >= 0 ? days : days - 146096) / 146097;
    const uint32_t dayOfEra = (uint32_t)(days - era * 146097);
    const uint32_t yearOfEra = (dayOfEra - dayOfEra / 1460U + dayOfEra / 36524U - dayOfEra / 146096U) / 365U;
    int32_t year = (int32_t)yearOfEra + era * 400;
    const uint32_t dayOfYear = dayOfEra - (365U * yearOfEra + yearOfEra / 4U - yearOfEra / 100U);
    const uint32_t monthPrime = (5U * dayOfYear + 2U) / 153U;
    const uint32_t resolvedDay = dayOfYear - (153U * monthPrime + 2U) / 5U + 1U;
    const uint32_t resolvedMonth = monthPrime + (monthPrime < 10U ? 3U : (uint32_t)-9);

    year += resolvedMonth <= 2U;

    if (fullYear != nullptr)
    {
        *fullYear = (uint16_t)year;
    }

    if (month != nullptr)
    {
        *month = (uint8_t)resolvedMonth;
    }

    if (day != nullptr)
    {
        *day = (uint8_t)resolvedDay;
    }
}

static time_t BuildEpochFromDateTime(uint16_t fullYear, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    const int32_t days = DaysFromCivil(fullYear, month, day);
    return (time_t)(((int64_t)days * 86400LL) + ((int64_t)hour * 3600LL) + ((int64_t)minute * 60LL) + (int64_t)second);
}

static void BreakEpochToDateTime(time_t epoch, uint16_t *fullYear, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    int64_t epochSeconds = (int64_t)epoch;
    int32_t days = (int32_t)(epochSeconds / 86400LL);
    int64_t secondsOfDay = epochSeconds % 86400LL;
    if (secondsOfDay < 0)
    {
        secondsOfDay += 86400LL;
        days -= 1;
    }

    CivilFromDays(days, fullYear, month, day);

    if (hour != nullptr)
    {
        *hour = (uint8_t)(secondsOfDay / 3600LL);
    }

    if (minute != nullptr)
    {
        *minute = (uint8_t)((secondsOfDay % 3600LL) / 60LL);
    }

    if (second != nullptr)
    {
        *second = (uint8_t)(secondsOfDay % 60LL);
    }
}

static uint8_t GetDayOfWeek(uint16_t fullYear, uint8_t month, uint8_t day)
{
    const int32_t days = DaysFromCivil(fullYear, month, day);
    int32_t dayOfWeek = (days + 4) % 7;
    if (dayOfWeek < 0)
    {
        dayOfWeek += 7;
    }

    return (uint8_t)dayOfWeek;
}

static bool IsFixedDateTransition(uint8_t encodedWeek)
{
    return (encodedWeek & TZ_RULE_FIXED_DATE_FLAG) != 0U;
}

static uint8_t GetFixedTransitionDay(uint8_t encodedWeek)
{
    return (uint8_t)(encodedWeek & TZ_RULE_DAY_MASK);
}

static uint8_t GetTransitionDayOfMonth(uint16_t fullYear, uint8_t month, uint8_t encodedWeek, uint8_t dayOfWeek)
{
    if (month < 1 || month > 12)
    {
        return 1;
    }

    if (IsFixedDateTransition(encodedWeek))
    {
        uint8_t day = GetFixedTransitionDay(encodedWeek);
        if (day < 1)
        {
            return 1;
        }

        const uint8_t daysInMonth = GetDaysInMonth(fullYear, month);
        if (day > daysInMonth)
        {
            day = daysInMonth;
        }

        return day;
    }

    const uint8_t week = encodedWeek;
    if (week == 0 || week > TZ_RULE_WEEK_LAST || dayOfWeek > 6)
    {
        return 1;
    }

    if (week == TZ_RULE_WEEK_LAST)
    {
        uint8_t day = GetDaysInMonth(fullYear, month);
        while (GetDayOfWeek(fullYear, month, day) != dayOfWeek && day > 1)
        {
            day--;
        }
        return day;
    }

    uint8_t day = 1;
    while (GetDayOfWeek(fullYear, month, day) != dayOfWeek)
    {
        day++;
    }

    day = (uint8_t)(day + (uint8_t)((week - 1U) * 7U));
    const uint8_t daysInMonth = GetDaysInMonth(fullYear, month);
    if (day > daysInMonth)
    {
        day = daysInMonth;
    }

    return day;
}

static bool IsValidTimeZoneRule(const TimeZoneRule &rule)
{
    if (rule.StandardOffsetMinutes < TZ_MIN_OFFSET_MINUTES || rule.StandardOffsetMinutes > TZ_MAX_OFFSET_MINUTES)
    {
        return false;
    }

    if (rule.DSTEnabled == 0)
    {
        return true;
    }

    if (rule.DSTOffsetMinutes <= 0 || rule.DSTOffsetMinutes > TZ_MAX_DST_OFFSET_MINUTES)
    {
        return false;
    }

    if (rule.DSTStartMonth < 1 || rule.DSTStartMonth > 12 || rule.DSTEndMonth < 1 || rule.DSTEndMonth > 12)
    {
        return false;
    }

    const bool startFixedDate = IsFixedDateTransition(rule.DSTStartWeek);
    const bool endFixedDate = IsFixedDateTransition(rule.DSTEndWeek);
    const uint8_t startWeekOrDay = startFixedDate ? GetFixedTransitionDay(rule.DSTStartWeek) : rule.DSTStartWeek;
    const uint8_t endWeekOrDay = endFixedDate ? GetFixedTransitionDay(rule.DSTEndWeek) : rule.DSTEndWeek;

    if (startWeekOrDay < 1 || endWeekOrDay < 1)
    {
        return false;
    }

    if (!startFixedDate && startWeekOrDay > TZ_RULE_WEEK_LAST)
    {
        return false;
    }

    if (!endFixedDate && endWeekOrDay > TZ_RULE_WEEK_LAST)
    {
        return false;
    }

    if (startFixedDate && startWeekOrDay > 31)
    {
        return false;
    }

    if (endFixedDate && endWeekOrDay > 31)
    {
        return false;
    }

    if ((!startFixedDate && rule.DSTStartDayOfWeek > 6) || (!endFixedDate && rule.DSTEndDayOfWeek > 6))
    {
        return false;
    }

    if (rule.DSTStartHour > 23 || rule.DSTEndHour > 23 || rule.DSTStartMinute > 59 || rule.DSTEndMinute > 59)
    {
        return false;
    }

    return true;
}

void SanitizeTimeZoneRule(TimeZoneRule *rule)
{
    if (rule == nullptr)
    {
        return;
    }

    if (rule->StandardOffsetMinutes < TZ_MIN_OFFSET_MINUTES)
    {
        rule->StandardOffsetMinutes = TZ_MIN_OFFSET_MINUTES;
    }
    else if (rule->StandardOffsetMinutes > TZ_MAX_OFFSET_MINUTES)
    {
        rule->StandardOffsetMinutes = TZ_MAX_OFFSET_MINUTES;
    }

    if (rule->DSTEnabled == 0)
    {
        rule->DSTOffsetMinutes = 0;
        rule->DSTStartMonth = 0;
        rule->DSTStartWeek = 0;
        rule->DSTStartDayOfWeek = 0;
        rule->DSTStartHour = 0;
        rule->DSTStartMinute = 0;
        rule->DSTEndMonth = 0;
        rule->DSTEndWeek = 0;
        rule->DSTEndDayOfWeek = 0;
        rule->DSTEndHour = 0;
        rule->DSTEndMinute = 0;
        return;
    }

    if (!IsValidTimeZoneRule(*rule))
    {
        memset(rule, 0, sizeof(TimeZoneRule));
    }
}

static void QueueSystemConfigSave()
{
    saveEEPROMOnTimeout = true;
    EEPROMSaveTimout = millis() + EEPROM_WRITE_DELAY;
}

static void UpdateRtcFromLocalEpoch(time_t localEpoch)
{
    rtc.setEpoch(localEpoch);
    RTCSet = true;

    if (!SDCardOK)
    {
        InitialiseSD();
    }
}

static time_t GetDstStartUtcEpoch(uint16_t fullYear)
{
    const uint8_t transitionDay = GetTransitionDayOfMonth(fullYear, SystemParams.TimeZone.DSTStartMonth, SystemParams.TimeZone.DSTStartWeek, SystemParams.TimeZone.DSTStartDayOfWeek);
    const time_t localEpoch = BuildEpochFromDateTime(fullYear,
                                                     SystemParams.TimeZone.DSTStartMonth,
                                                     transitionDay,
                                                     SystemParams.TimeZone.DSTStartHour,
                                                     SystemParams.TimeZone.DSTStartMinute,
                                                     0);
    return (time_t)(localEpoch - ((time_t)SystemParams.TimeZone.StandardOffsetMinutes * 60));
}

static time_t GetDstEndUtcEpoch(uint16_t fullYear)
{
    const uint8_t transitionDay = GetTransitionDayOfMonth(fullYear, SystemParams.TimeZone.DSTEndMonth, SystemParams.TimeZone.DSTEndWeek, SystemParams.TimeZone.DSTEndDayOfWeek);
    const time_t localEpoch = BuildEpochFromDateTime(fullYear,
                                                     SystemParams.TimeZone.DSTEndMonth,
                                                     transitionDay,
                                                     SystemParams.TimeZone.DSTEndHour,
                                                     SystemParams.TimeZone.DSTEndMinute,
                                                     0);
    return (time_t)(localEpoch - ((time_t)(SystemParams.TimeZone.StandardOffsetMinutes + SystemParams.TimeZone.DSTOffsetMinutes) * 60));
}

static bool IsDSTActiveForUtcEpoch(time_t utcEpoch)
{
    if (!IsValidTimeZoneRule(SystemParams.TimeZone) || SystemParams.TimeZone.DSTEnabled == 0)
    {
        return false;
    }

    uint16_t fullYear = 0;
    BreakEpochToDateTime(utcEpoch, &fullYear, nullptr, nullptr, nullptr, nullptr, nullptr);

    const time_t startUtc = GetDstStartUtcEpoch(fullYear);
    const time_t endUtc = GetDstEndUtcEpoch(fullYear);

    if (startUtc < endUtc)
    {
        return utcEpoch >= startUtc && utcEpoch < endUtc;
    }

    if (utcEpoch < endUtc)
    {
        return true;
    }

    return utcEpoch >= startUtc;
}

static bool DetermineDstStateFromLocalEpoch(time_t localEpoch)
{
    if (!IsValidTimeZoneRule(SystemParams.TimeZone) || SystemParams.TimeZone.DSTEnabled == 0)
    {
        return false;
    }

    const time_t standardUtcEpoch = (time_t)(localEpoch - (time_t)(SystemParams.TimeZone.StandardOffsetMinutes * 60L));
    const time_t dstUtcEpoch = (time_t)(localEpoch - (time_t)((SystemParams.TimeZone.StandardOffsetMinutes + SystemParams.TimeZone.DSTOffsetMinutes) * 60L));
    const bool standardCandidateValid = !IsDSTActiveForUtcEpoch(standardUtcEpoch);
    const bool dstCandidateValid = IsDSTActiveForUtcEpoch(dstUtcEpoch);

    if (dstCandidateValid && !standardCandidateValid)
    {
        return true;
    }

    if (standardCandidateValid && !dstCandidateValid)
    {
        return false;
    }

    if (dstCandidateValid && standardCandidateValid)
    {
        return SystemParams.DSTActive != 0;
    }

    return false;
}

static void UpdateAutomaticDSTAdjustment()
{
    if (!RTCSet || !HasUsableRtcTime())
    {
        return;
    }

    if (!IsValidTimeZoneRule(SystemParams.TimeZone) || SystemParams.TimeZone.DSTEnabled == 0)
    {
        if (SystemParams.DSTActive != 0)
        {
            SystemParams.DSTActive = 0;
            QueueSystemConfigSave();
        }
        return;
    }

    const int32_t appliedOffsetMinutes = (int32_t)SystemParams.TimeZone.StandardOffsetMinutes + ((SystemParams.DSTActive != 0) ? (int32_t)SystemParams.TimeZone.DSTOffsetMinutes : 0);
    const time_t localEpoch = rtc.getEpoch();
    const time_t utcEpoch = (time_t)(localEpoch - (time_t)(appliedOffsetMinutes * 60L));
    const bool shouldBeDst = IsDSTActiveForUtcEpoch(utcEpoch);
    const bool isDst = SystemParams.DSTActive != 0;

    if (shouldBeDst == isDst)
    {
        return;
    }

    const int32_t deltaSeconds = (int32_t)SystemParams.TimeZone.DSTOffsetMinutes * 60L;
    UpdateRtcFromLocalEpoch((time_t)(localEpoch + (shouldBeDst ? deltaSeconds : -deltaSeconds)));
    SystemParams.DSTActive = shouldBeDst ? 1 : 0;
    QueueSystemConfigSave();
}

static ThermalProtectionStage DetermineThermalProtectionStage(float hottestTemperature, ThermalProtectionStage currentStage)
{
    switch (currentStage)
    {
    case THERMAL_PROTECTION_ERROR:
        if (hottestTemperature >= SYSTEM_TEMP_ERROR_CLEAR_THRESHOLD)
        {
            return THERMAL_PROTECTION_ERROR;
        }
        if (hottestTemperature >= SYSTEM_TEMP_WARNING_THRESHOLD)
        {
            return THERMAL_PROTECTION_WARNING;
        }
        return THERMAL_PROTECTION_NONE;

    case THERMAL_PROTECTION_WARNING:
        if (hottestTemperature >= SYSTEM_TEMP_ERROR_THRESHOLD)
        {
            return THERMAL_PROTECTION_ERROR;
        }
        if (hottestTemperature >= SYSTEM_TEMP_WARNING_CLEAR_THRESHOLD)
        {
            return THERMAL_PROTECTION_WARNING;
        }
        return THERMAL_PROTECTION_NONE;

    default:
        if (hottestTemperature >= SYSTEM_TEMP_ERROR_THRESHOLD)
        {
            return THERMAL_PROTECTION_ERROR;
        }
        if (hottestTemperature >= SYSTEM_TEMP_WARNING_THRESHOLD)
        {
            return THERMAL_PROTECTION_WARNING;
        }
        return THERMAL_PROTECTION_NONE;
    }
}

void IgnitionWake()
{
    if (PowerState != SLEEPING)
    {
        return;
    }

    ignitionWakePending = true;
}

void IMUWake()
{
    if (PowerState != SLEEPING)
    {
        return;
    }

    if (!IMUWakeMode)
    {
        IMUWakeMode = true;
        imuWakePending = true;
    }
}

void InitialiseSystem()
{
    // Start the low power features. Attach sleep mode interrupts
    LowPower.begin();
    LowPower.attachInterruptWakeup(IMU_INT1, IMUWake, RISING, DEEP_SLEEP_MODE);
    LowPower.attachInterruptWakeup(IGN_INPUT, IgnitionWake, RISING, DEEP_SLEEP_MODE);

    // Set power state to run
    PowerState = RUN;

    WakeSystem();

    // Debug pin
    pinMode(DEBUG_PIN, OUTPUT);

    // Spare I/O as inputs
    pinMode(PA8, INPUT_ANALOG);
    pinMode(PC13, INPUT_ANALOG);
    pinMode(PD10, INPUT_ANALOG);
    pinMode(PD10, INPUT_ANALOG);
    pinMode(PD11, INPUT_ANALOG);
    pinMode(PD12, INPUT_ANALOG);
    pinMode(PD13, INPUT_ANALOG);
    pinMode(PD15, INPUT_ANALOG);
    pinMode(PE6, INPUT_ANALOG);
    pinMode(PG7, INPUT_ANALOG);
    pinMode(PG8, INPUT_ANALOG);

    pinMode(CHARGE_EN, OUTPUT);
    pinMode(BATT_INT, INPUT);

    digitalWrite(CHARGE_EN, HIGH); // Active low

    // SPI
    pinMode(CS1, OUTPUT);
    pinMode(CS2, OUTPUT);
    digitalWrite(CS1, HIGH);
    digitalWrite(CS2, HIGH);

    RTCSet = false;

    // I2C
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);

    pinMode(SIM_PWR, OUTPUT);
    pinMode(SIM_RST, OUTPUT);
    pinMode(SIM_FLIGHT, OUTPUT);
    digitalWrite(SIM_PWR, LOW);
    digitalWrite(SIM_RST, LOW);
    digitalWrite(SIM_FLIGHT, LOW);

    // SIM module power
    pinMode(SIM_REGULATOR, OUTPUT);
    digitalWrite(SIM_REGULATOR, HIGH); 
}

void InitialiseSystemData()
{
    // Initialise default system data
    memset(&SystemParams, 0, sizeof(SystemParams));
    SystemParams.CANResEnabled = 1;
    SystemParams.ChannelDataCANID = CHAN_CAN_ID;
    SystemParams.DigitalInputDataCANID = DIG_INPUT_CAN_ID;
    SystemParams.AnalogueInputDataCANID = ANA_INPUT_CAN_ID;
    SystemParams.SystemDataCANID = SYS_CAN_ID;
    SystemParams.SystemConfigDataCANID = SYS_CONFIG_CAN_ID;
    SystemParams.ChannelConfigDataCANID = CONF_CAN_ID;
    SystemParams.IMUwakeWindow = DEFAULT_WW;
    SystemParams.MotionDeadTime = DEFAULT_MOTION_DEADTIME;
    SystemParams.SystemCurrentLimit = SYSTEM_CURRENT_MAX;
    SystemParams.AllowData = 1;
    SystemParams.AllowGPS = 1;
    SystemParams.SpeedUnitPref = 1;
    SystemParams.DistanceUnitPref = 1;
    SystemParams.AllowMotionDetect = 1;
    SystemParams.DSTActive = 0;
    SystemParams.CANBusBitrate = DEFAULT_CAN_BUS_BITRATE;
}

bool IsSupportedCANBusBitrate(uint32_t bitrate)
{
    switch (bitrate)
    {
    case CAN_BUS_BITRATE_125K:
    case CAN_BUS_BITRATE_250K:
    case CAN_BUS_BITRATE_500K:
    case CAN_BUS_BITRATE_1M:
        return true;
    default:
        return false;
    }
}

bool IsValidRtcDateTime(uint16_t fullYear, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    if (fullYear < RTC_MIN_FULL_YEAR || fullYear > RTC_MAX_FULL_YEAR)
    {
        return false;
    }

    if (month < 1 || month > 12)
    {
        return false;
    }

    if (day < 1 || day > 31)
    {
        return false;
    }

    if (hour > 23 || minute > 59 || second > 59)
    {
        return false;
    }

    return true;
}

bool HasUsableRtcTime()
{
    return rtc.isTimeSet() && rtc.getYear() >= RTC_MIN_VALID_YEAR;
}

bool ApplyRtcDateTime(uint16_t fullYear, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    if (!IsValidRtcDateTime(fullYear, month, day, hour, minute, second))
    {
        return false;
    }

    const time_t localEpoch = BuildEpochFromDateTime(fullYear, month, day, hour, minute, second);
    UpdateRtcFromLocalEpoch(localEpoch);

    const uint8_t dstActive = DetermineDstStateFromLocalEpoch(localEpoch) ? 1 : 0;
    if (SystemParams.DSTActive != dstActive)
    {
        SystemParams.DSTActive = dstActive;
        QueueSystemConfigSave();
    }
    else
    {
        SystemParams.DSTActive = dstActive;
    }

    return true;
}

bool ApplyUtcRtcDateTime(uint16_t fullYear, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    if (!IsValidRtcDateTime(fullYear, month, day, hour, minute, second))
    {
        return false;
    }

    SanitizeTimeZoneRule(&SystemParams.TimeZone);

    const time_t utcEpoch = BuildEpochFromDateTime(fullYear, month, day, hour, minute, second);
    const bool dstActive = IsDSTActiveForUtcEpoch(utcEpoch);
    const int32_t localOffsetMinutes = (int32_t)SystemParams.TimeZone.StandardOffsetMinutes + (dstActive ? (int32_t)SystemParams.TimeZone.DSTOffsetMinutes : 0);
    const time_t targetLocalEpoch = (time_t)(utcEpoch + (time_t)(localOffsetMinutes * 60L));
    const bool currentDstActive = SystemParams.DSTActive != 0;

    if (RTCSet && HasUsableRtcTime() && rtc.getEpoch() == targetLocalEpoch && currentDstActive == dstActive)
    {
        return true;
    }

    UpdateRtcFromLocalEpoch(targetLocalEpoch);

    if (currentDstActive != dstActive)
    {
        SystemParams.DSTActive = dstActive ? 1 : 0;
        QueueSystemConfigSave();
    }
    else
    {
        SystemParams.DSTActive = dstActive ? 1 : 0;
    }

    return true;
}

void UpdateSystem()
{
    // Get system temperature
    int32_t VRef = readVref();
    SystemRuntimeParams.SystemTemperature = readTempSensor(VRef);
    SystemRuntimeParams.SIMModuleTemp = simModuleTemp;
    SystemRuntimeParams.IMUTemp = imuTemp;

    float hottestTemperature = (float)SystemRuntimeParams.SystemTemperature;
    if (SystemRuntimeParams.IMUTemp > hottestTemperature)
    {
        hottestTemperature = SystemRuntimeParams.IMUTemp;
    }

    ThermalProtectionStage previousThermalStage = ActiveThermalProtectionStage;
    ActiveThermalProtectionStage = DetermineThermalProtectionStage(hottestTemperature, ActiveThermalProtectionStage);
    if (previousThermalStage != ActiveThermalProtectionStage)
    {
        invalidateDisplay = true;
    }

    // Calculate battery voltage
    SystemRuntimeParams.VBatt = analogRead(VBATT_ANALOG_PIN) * 0.0039787f;

    // Calculate system current draw

    UpdateAutomaticDSTAdjustment();
    SystemRuntimeParams.SystemCurrent = 0.0f;
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        SystemRuntimeParams.SystemCurrent += ChannelRuntime[i].CurrentValue;
    }

    // Check system temperature limit
    if (SystemRuntimeParams.SystemTemperature > SYSTEM_TEMP_LIMIT)
    {
        SystemRuntimeParams.ErrorFlags |= OVERTEMP;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~OVERTEMP;
    }

    if (ActiveThermalProtectionStage >= THERMAL_PROTECTION_WARNING)
    {
        SystemRuntimeParams.ErrorFlags |= TEMP_WARNING;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~TEMP_WARNING;
    }

    if (ActiveThermalProtectionStage >= THERMAL_PROTECTION_ERROR)
    {
        SystemRuntimeParams.ErrorFlags |= OVERTEMP;
    }

    // Check battery voltage
    if (SystemRuntimeParams.VBatt <= LOGGING_VBATT_THRESHOLD)
    {
        SystemRuntimeParams.ErrorFlags |= UNDERVOLTAGE;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~UNDERVOLTAGE;
    }

    // Check current limit
    if (SystemRuntimeParams.SystemCurrent > SYSTEM_CURRENT_MAX)
    {
        SystemRuntimeParams.ErrorFlags |= OVERCURRENT;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~OVERCURRENT;
    }

    // Check CRC
    if (!SystemCRCValid)
    {
        SystemRuntimeParams.ErrorFlags |= CRC_CHECK_FAILED;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~CRC_CHECK_FAILED;
    }

    // Check SD card status
    if (!SDCardOK)
    {
        SystemRuntimeParams.ErrorFlags |= SDCARD_ERROR;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~SDCARD_ERROR;
    }

    // Check GPS fix
    if (!GPSFix && SystemParams.AllowGPS)
    {
        SystemRuntimeParams.ErrorFlags |= GPS_ERROR;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~GPS_ERROR;
    }

    if (IsTelemetryOffline())
    {
        SystemRuntimeParams.ErrorFlags |= TELEMETRY_OFFLINE;
    }
    else
    {
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~TELEMETRY_OFFLINE;
    }

    // Checksum status of PC communications
    switch (connectionStatus)
    {
    default:
        // Disconnected or connected successfully
        SystemRuntimeParams.ErrorFlags = SystemRuntimeParams.ErrorFlags & ~PC_COMMS_CHECKSUM_ERROR;
        break;
    case 8:
        SystemRuntimeParams.ErrorFlags |= PC_COMMS_CHECKSUM_ERROR;
        break;
    }
}

void SleepSystem()
{
    // Power down peripherals
    pinMode(PWR_EN_5V, OUTPUT);
    pinMode(PWR_EN_3V3, OUTPUT);
    digitalWrite(PWR_EN_5V, LOW);
    digitalWrite(PWR_EN_3V3, LOW);

    pinMode(SIM_PWR, OUTPUT);
    pinMode(SIM_RST, OUTPUT);
    pinMode(SIM_FLIGHT, OUTPUT);
    digitalWrite(SIM_PWR, LOW);
    digitalWrite(SIM_RST, LOW);
    digitalWrite(SIM_FLIGHT, LOW);

    // Power down SIM7600
    digitalWrite(SIM_REGULATOR, LOW); 
}

void WakeSystem()
{
    // Set the analogue read resolution
    analogReadResolution(12);
    analogWriteResolution(10);

    // Keep the display dark until a wake path explicitly enables it.
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);

    pinMode(SIM_PWR, OUTPUT);
    pinMode(SIM_RST, OUTPUT);
    pinMode(SIM_FLIGHT, OUTPUT);
    digitalWrite(SIM_PWR, LOW);
    digitalWrite(SIM_RST, LOW);
    digitalWrite(SIM_FLIGHT, LOW);

    // Power up peripherals
    pinMode(PWR_EN_5V, OUTPUT);
    pinMode(PWR_EN_3V3, OUTPUT);
    digitalWrite(PWR_EN_5V, HIGH);
    digitalWrite(PWR_EN_3V3, HIGH);

    // Power up SIM7600
    digitalWrite(SIM_REGULATOR, HIGH);
}

static int32_t readTempSensor(int32_t VRef)
{
    return (__LL_ADC_CALC_TEMPERATURE(VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
}

static int32_t readVref()
{
    return (__LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION));
}
