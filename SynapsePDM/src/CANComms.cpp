/*  CANComms.h CAN bus variables, functions and data handling.
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

#include "CANComms.h"
#include "InputHandler.h"

// Use CAN1 with ALT_2 pin configuration (PD0/PD1)
STM32_CAN Can(CAN1, ALT_2);

static uint8_t BuildDigitalInputMask()
{
    uint8_t mask = 0;

    for (int i = 0; i < NUM_DI_CHANNELS; i++)
    {
        if (digitalRead(DIchannelInputPins[i]))
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

static void StoreUint16BigEndian(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)((value >> 8) & 0xFF);
    buffer[1] = (uint8_t)(value & 0xFF);
}

static void StoreInt32BigEndian(uint8_t *buffer, int32_t value)
{
    buffer[0] = (uint8_t)((value >> 24) & 0xFF);
    buffer[1] = (uint8_t)((value >> 16) & 0xFF);
    buffer[2] = (uint8_t)((value >> 8) & 0xFF);
    buffer[3] = (uint8_t)(value & 0xFF);
}

static void StoreUint32BigEndian(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)((value >> 24) & 0xFF);
    buffer[1] = (uint8_t)((value >> 16) & 0xFF);
    buffer[2] = (uint8_t)((value >> 8) & 0xFF);
    buffer[3] = (uint8_t)(value & 0xFF);
}

static uint16_t LoadUint16BigEndian(const uint8_t *buffer)
{
    return (uint16_t)(((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
}

static uint32_t LoadUint32BigEndian(const uint8_t *buffer)
{
    return ((uint32_t)buffer[0] << 24) |
           ((uint32_t)buffer[1] << 16) |
           ((uint32_t)buffer[2] << 8) |
           (uint32_t)buffer[3];
}

static float LoadFloatBigEndian(const uint8_t *buffer)
{
    uint32_t raw = LoadUint32BigEndian(buffer);
    float value = 0.0f;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static bool IsTemperatureUnitsForCAN(uint8_t units)
{
    return units == ANA_UNITS_CELSIUS || units == ANA_UNITS_FAHRENHEIT;
}

static float ConvertTemperatureValueForCAN(float value, uint8_t fromUnits, uint8_t toUnits)
{
    if (fromUnits == toUnits)
    {
        return value;
    }

    if (fromUnits == ANA_UNITS_CELSIUS && toUnits == ANA_UNITS_FAHRENHEIT)
    {
        return (value * 1.8f) + 32.0f;
    }

    if (fromUnits == ANA_UNITS_FAHRENHEIT && toUnits == ANA_UNITS_CELSIUS)
    {
        return (value - 32.0f) / 1.8f;
    }

    return value;
}

static void ScheduleCANConfigSave()
{
    saveEEPROMOnTimeout = true;
    invalidateDisplay = true;
    EEPROMSaveTimout = millis() + EEPROM_WRITE_DELAY;
}

static bool pendingCANBitrateReinitialiseOnSave = false;
static uint8_t pendingTimeZoneConfigMask = 0;
static uint8_t pendingTimeZoneConfigBytes[sizeof(TimeZoneRule)] = {0};

static bool ApplyExtendedChannelParameter(uint8_t channelIndex, uint8_t parameter, const uint8_t *payload, bool *inputConfigChanged)
{
    if (channelIndex >= NUM_CHANNELS || payload == nullptr)
    {
        return false;
    }

    bool configChanged = false;

    switch (parameter)
    {
    case 0:
    {
        if (payload[0] > 6)
        {
            return false;
        }

        ChannelType newType = (ChannelType)payload[0];
        if (newType != Channels[channelIndex].ChanType)
        {
            if (Channels[channelIndex].ChanType != DIG_INTERMITTENT && newType == DIG_INTERMITTENT)
            {
                Channels[channelIndex].IntermittentOnTime = 1000;
                Channels[channelIndex].IntermittentOffTime = 1000;
            }

            Channels[channelIndex].ChanType = newType;
            configChanged = true;
            if (inputConfigChanged != nullptr)
            {
                *inputConfigChanged = true;
            }
        }
        break;
    }
    case 1:
        ChannelRuntime[channelIndex].Override = payload[0];
        return true;
    case 2:
    {
        float threshold = LoadFloatBigEndian(payload);
        if (threshold > CURRENT_MAX)
        {
            threshold = CURRENT_MAX;
        }
        if (threshold < 0.0f)
        {
            threshold = 0.0f;
        }
        if (threshold != Channels[channelIndex].CurrentThresholdHigh)
        {
            Channels[channelIndex].CurrentThresholdHigh = threshold;
            configChanged = true;
        }
        break;
    }
    case 3:
    {
        float threshold = LoadFloatBigEndian(payload);
        if (threshold < 0.0f)
        {
            threshold = 0.0f;
        }
        if (threshold > CURRENT_MAX)
        {
            threshold = CURRENT_MAX;
        }
        if (threshold != Channels[channelIndex].CurrentThresholdLow)
        {
            Channels[channelIndex].CurrentThresholdLow = threshold;
            configChanged = true;
        }
        break;
    }
    case 4:
        if (Channels[channelIndex].Enabled != payload[0])
        {
            Channels[channelIndex].Enabled = payload[0];
            configChanged = true;
        }
        break;
    case 5:
        if (Channels[channelIndex].GroupNumber != payload[0])
        {
            Channels[channelIndex].GroupNumber = payload[0];
            configChanged = true;
        }
        break;
    case 6:
        if (Channels[channelIndex].InputControlPin != payload[0])
        {
            Channels[channelIndex].InputControlPin = payload[0];
            configChanged = true;
        }
        if (SyncChannelTypeForAssignedInput(channelIndex))
        {
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 7:
        if (Channels[channelIndex].MultiChannel != payload[0])
        {
            Channels[channelIndex].MultiChannel = payload[0];
            configChanged = true;
        }
        break;
    case 8:
        if (Channels[channelIndex].RetryCount != payload[0])
        {
            Channels[channelIndex].RetryCount = payload[0];
            configChanged = true;
        }
        break;
    case 9:
    {
        uint32_t inrushDelay = LoadUint32BigEndian(payload);
        if (inrushDelay > INRUSH_MAX)
        {
            inrushDelay = INRUSH_MAX;
        }
        if (inrushDelay != Channels[channelIndex].InrushDelay)
        {
            Channels[channelIndex].InrushDelay = inrushDelay;
            configChanged = true;
        }
        break;
    }
    case 10:
        if (memcmp(Channels[channelIndex].ChannelName, payload, sizeof(Channels[channelIndex].ChannelName)) != 0)
        {
            memcpy(Channels[channelIndex].ChannelName, payload, sizeof(Channels[channelIndex].ChannelName));
            configChanged = true;
        }
        break;
    case 11:
    {
        uint8_t oldDelayedOff = Channels[channelIndex].DelayedOff;
        uint8_t oldTrigger = Channels[channelIndex].DelayedOffTrigger;
        uint32_t oldTime = Channels[channelIndex].DelayedOffTime;
        if (payload[0])
        {
            Channels[channelIndex].DelayedOff = 1;
            Channels[channelIndex].DelayedOffTrigger = DELAYED_OFF_IGNITION_OFF;
            if (Channels[channelIndex].DelayedOffTime < MIN_DELAY_TIME_MS)
            {
                Channels[channelIndex].DelayedOffTime = MIN_DELAY_TIME_MS;
            }
        }
        if (oldDelayedOff != Channels[channelIndex].DelayedOff ||
            oldTrigger != Channels[channelIndex].DelayedOffTrigger ||
            oldTime != Channels[channelIndex].DelayedOffTime)
        {
            configChanged = true;
        }
        break;
    }
    case 12:
    {
        uint32_t delayedOffTime = LoadUint32BigEndian(payload);
        if (delayedOffTime > MAX_DELAY_TIME_MS)
        {
            delayedOffTime = MAX_DELAY_TIME_MS;
        }

        uint8_t oldDelayedOff = Channels[channelIndex].DelayedOff;
        uint8_t oldTrigger = Channels[channelIndex].DelayedOffTrigger;
        uint32_t oldTime = Channels[channelIndex].DelayedOffTime;

        Channels[channelIndex].DelayedOffTime = delayedOffTime;
        if (Channels[channelIndex].DelayedOffTime > 0)
        {
            Channels[channelIndex].DelayedOff = 1;
            Channels[channelIndex].DelayedOffTrigger = DELAYED_OFF_IGNITION_OFF;
        }

        if (oldDelayedOff != Channels[channelIndex].DelayedOff ||
            oldTrigger != Channels[channelIndex].DelayedOffTrigger ||
            oldTime != Channels[channelIndex].DelayedOffTime)
        {
            configChanged = true;
        }
        break;
    }
    case 13:
        if (Channels[channelIndex].SoftStart != payload[0])
        {
            Channels[channelIndex].SoftStart = payload[0];
            configChanged = true;
        }
        break;
    case 14:
    {
        uint32_t softStartTime = LoadUint32BigEndian(payload);
        if (softStartTime > MAX_SOFT_START_TIME)
        {
            softStartTime = MAX_SOFT_START_TIME;
        }
        if (softStartTime != Channels[channelIndex].SoftStartTime)
        {
            Channels[channelIndex].SoftStartTime = softStartTime;
            configChanged = true;
        }
        break;
    }
    case 15:
    {
        float inrushCurrentThreshold = LoadFloatBigEndian(payload);
        if (inrushCurrentThreshold > INRUSH_CURRENT_MAX)
        {
            inrushCurrentThreshold = INRUSH_CURRENT_MAX;
        }
        if (inrushCurrentThreshold < 0.0f)
        {
            inrushCurrentThreshold = 0.0f;
        }
        if (inrushCurrentThreshold != Channels[channelIndex].InrushCurrentThreshold)
        {
            Channels[channelIndex].InrushCurrentThreshold = inrushCurrentThreshold;
            configChanged = true;
        }
        break;
    }
    case 16:
        if (Channels[channelIndex].PWMSetDuty != payload[0])
        {
            Channels[channelIndex].PWMSetDuty = payload[0];
            configChanged = true;
        }
        break;
    case 17:
    {
        float onThreshold = LoadFloatBigEndian(payload);
        if (onThreshold != Channels[channelIndex].OnThreshold)
        {
            Channels[channelIndex].OnThreshold = onThreshold;
            configChanged = true;
        }
        break;
    }
    case 18:
    {
        float offThreshold = LoadFloatBigEndian(payload);
        if (offThreshold != Channels[channelIndex].OffThreshold)
        {
            Channels[channelIndex].OffThreshold = offThreshold;
            configChanged = true;
        }
        break;
    }
    case 19:
    {
        float scaleMin = LoadFloatBigEndian(payload);
        if (scaleMin != Channels[channelIndex].ScaleMin)
        {
            Channels[channelIndex].ScaleMin = scaleMin;
            configChanged = true;
        }
        break;
    }
    case 20:
    {
        float scaleMax = LoadFloatBigEndian(payload);
        if (scaleMax != Channels[channelIndex].ScaleMax)
        {
            Channels[channelIndex].ScaleMax = scaleMax;
            configChanged = true;
        }
        break;
    }
    case 21:
        if (Channels[channelIndex].PWMMin != payload[0])
        {
            Channels[channelIndex].PWMMin = payload[0];
            configChanged = true;
        }
        break;
    case 22:
        if (Channels[channelIndex].PWMMax != payload[0])
        {
            Channels[channelIndex].PWMMax = payload[0];
            configChanged = true;
        }
        break;
    case 23:
        if (Channels[channelIndex].SoftStop != payload[0])
        {
            Channels[channelIndex].SoftStop = payload[0];
            configChanged = true;
        }
        break;
    case 24:
    {
        uint32_t softStopTime = LoadUint32BigEndian(payload);
        if (softStopTime > MAX_SOFT_STOP_TIME)
        {
            softStopTime = MAX_SOFT_STOP_TIME;
        }
        if (softStopTime != Channels[channelIndex].SoftStopTime)
        {
            Channels[channelIndex].SoftStopTime = softStopTime;
            configChanged = true;
        }
        break;
    }
    case 25:
    {
        ChannelCategory category = SanitizeChannelCategory(payload[0]);
        if (category != Channels[channelIndex].Category)
        {
            Channels[channelIndex].Category = category;
            configChanged = true;
        }
        break;
    }
    case 26:
    {
        uint32_t intermittentOnTime = LoadUint32BigEndian(payload);
        if (intermittentOnTime > MAX_INTERMITTENT_TIME)
        {
            intermittentOnTime = MAX_INTERMITTENT_TIME;
        }
        if (intermittentOnTime != Channels[channelIndex].IntermittentOnTime)
        {
            Channels[channelIndex].IntermittentOnTime = intermittentOnTime;
            configChanged = true;
        }
        break;
    }
    case 27:
    {
        uint32_t intermittentOffTime = LoadUint32BigEndian(payload);
        if (intermittentOffTime > MAX_INTERMITTENT_TIME)
        {
            intermittentOffTime = MAX_INTERMITTENT_TIME;
        }
        if (intermittentOffTime != Channels[channelIndex].IntermittentOffTime)
        {
            Channels[channelIndex].IntermittentOffTime = intermittentOffTime;
            configChanged = true;
        }
        break;
    }
    case 28:
    {
        uint8_t oldDelayedOn = Channels[channelIndex].DelayedOn;
        uint32_t oldTime = Channels[channelIndex].DelayedOnTime;
        Channels[channelIndex].DelayedOn = payload[0] ? 1 : 0;
        if (Channels[channelIndex].DelayedOn && Channels[channelIndex].DelayedOnTime < MIN_DELAY_TIME_MS)
        {
            Channels[channelIndex].DelayedOnTime = MIN_DELAY_TIME_MS;
        }
        if (oldDelayedOn != Channels[channelIndex].DelayedOn || oldTime != Channels[channelIndex].DelayedOnTime)
        {
            configChanged = true;
        }
        break;
    }
    case 29:
    {
        uint32_t delayedOnTime = LoadUint32BigEndian(payload);
        if (delayedOnTime > MAX_DELAY_TIME_MS)
        {
            delayedOnTime = MAX_DELAY_TIME_MS;
        }
        if (Channels[channelIndex].DelayedOn && delayedOnTime < MIN_DELAY_TIME_MS)
        {
            delayedOnTime = MIN_DELAY_TIME_MS;
        }
        if (delayedOnTime != Channels[channelIndex].DelayedOnTime)
        {
            Channels[channelIndex].DelayedOnTime = delayedOnTime;
            configChanged = true;
        }
        break;
    }
    case 30:
    {
        uint8_t oldDelayedOff = Channels[channelIndex].DelayedOff;
        uint32_t oldTime = Channels[channelIndex].DelayedOffTime;
        Channels[channelIndex].DelayedOff = payload[0] ? 1 : 0;
        if (Channels[channelIndex].DelayedOff && Channels[channelIndex].DelayedOffTime < MIN_DELAY_TIME_MS)
        {
            Channels[channelIndex].DelayedOffTime = MIN_DELAY_TIME_MS;
        }
        if (oldDelayedOff != Channels[channelIndex].DelayedOff || oldTime != Channels[channelIndex].DelayedOffTime)
        {
            configChanged = true;
        }
        break;
    }
    case 31:
    {
        uint32_t delayedOffTime = LoadUint32BigEndian(payload);
        if (delayedOffTime > MAX_DELAY_TIME_MS)
        {
            delayedOffTime = MAX_DELAY_TIME_MS;
        }
        if (Channels[channelIndex].DelayedOff && delayedOffTime < MIN_DELAY_TIME_MS)
        {
            delayedOffTime = MIN_DELAY_TIME_MS;
        }
        if (delayedOffTime != Channels[channelIndex].DelayedOffTime)
        {
            Channels[channelIndex].DelayedOffTime = delayedOffTime;
            configChanged = true;
        }
        break;
    }
    case 32:
    {
        uint8_t delayedOffTrigger = (payload[0] <= DELAYED_OFF_IGNITION_OFF) ? payload[0] : DELAYED_OFF_ASSIGNED_INPUT;
        if (delayedOffTrigger != Channels[channelIndex].DelayedOffTrigger)
        {
            Channels[channelIndex].DelayedOffTrigger = delayedOffTrigger;
            configChanged = true;
        }
        break;
    }
    default:
        return false;
    }

    if (configChanged)
    {
        pendingEEPROMSave = true;
    }

    return true;
}

static bool ApplyExtendedAnalogueParameter(uint8_t inputIndex, uint8_t parameter, const uint8_t *payload, bool *inputConfigChanged)
{
    if (inputIndex >= NUM_ANA_CHANNELS || payload == nullptr)
    {
        return false;
    }

    bool configChanged = false;

    switch (parameter)
    {
    case 0:
        if (AnalogueIns[inputIndex].PullUpEnable != (payload[0] != 0))
        {
            AnalogueIns[inputIndex].PullUpEnable = payload[0] != 0;
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 1:
        if (AnalogueIns[inputIndex].PullDownEnable != (payload[0] != 0))
        {
            AnalogueIns[inputIndex].PullDownEnable = payload[0] != 0;
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 2:
        AnalogueIns[inputIndex].ChanType = (AnalogueChannelType)payload[0];
        if (AnalogueIns[inputIndex].ChanType == NTC)
        {
            AnalogueIns[inputIndex].PullUpEnable = true;
            AnalogueIns[inputIndex].PullDownEnable = false;
            AnalogueIns[inputIndex].NTCNominalResistance = 10000.0f;
        }
        configChanged = true;
        if (SyncChannelTypesForAnalogueInput(inputIndex))
        {
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 3:
    {
        uint8_t previousUnits = AnalogueIns[inputIndex].Units;
        AnalogueIns[inputIndex].Units = (AnalogueUnits)payload[0];

        if (IsTemperatureUnitsForCAN(previousUnits) &&
            IsTemperatureUnitsForCAN(AnalogueIns[inputIndex].Units) &&
            previousUnits != AnalogueIns[inputIndex].Units)
        {
            uint8_t inputPin = ANAchannelInputPins[inputIndex];
            for (int channel = 0; channel < NUM_CHANNELS; channel++)
            {
                if (Channels[channel].InputControlPin != inputPin)
                {
                    continue;
                }

                switch (Channels[channel].ChanType)
                {
                case ANA:
                    Channels[channel].OnThreshold = ConvertTemperatureValueForCAN(Channels[channel].OnThreshold, previousUnits, AnalogueIns[inputIndex].Units);
                    Channels[channel].OffThreshold = ConvertTemperatureValueForCAN(Channels[channel].OffThreshold, previousUnits, AnalogueIns[inputIndex].Units);
                    break;
                case ANA_PWM:
                    Channels[channel].ScaleMin = ConvertTemperatureValueForCAN(Channels[channel].ScaleMin, previousUnits, AnalogueIns[inputIndex].Units);
                    Channels[channel].ScaleMax = ConvertTemperatureValueForCAN(Channels[channel].ScaleMax, previousUnits, AnalogueIns[inputIndex].Units);
                    break;
                default:
                    break;
                }
            }
        }

        configChanged = true;
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    }
    case 4:
        if (AnalogueIns[inputIndex].CalibrationPoints != payload[0])
        {
            AnalogueIns[inputIndex].CalibrationPoints = payload[0];
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 5:
        if (AnalogueIns[inputIndex].CalibrationVolt1 != LoadFloatBigEndian(payload))
        {
            AnalogueIns[inputIndex].CalibrationVolt1 = LoadFloatBigEndian(payload);
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 6:
        if (AnalogueIns[inputIndex].CalibrationValue1 != LoadFloatBigEndian(payload))
        {
            AnalogueIns[inputIndex].CalibrationValue1 = LoadFloatBigEndian(payload);
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 7:
        if (AnalogueIns[inputIndex].CalibrationVolt2 != LoadFloatBigEndian(payload))
        {
            AnalogueIns[inputIndex].CalibrationVolt2 = LoadFloatBigEndian(payload);
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 8:
        if (AnalogueIns[inputIndex].CalibrationValue2 != LoadFloatBigEndian(payload))
        {
            AnalogueIns[inputIndex].CalibrationValue2 = LoadFloatBigEndian(payload);
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 9:
        if (AnalogueIns[inputIndex].CalibrationVolt3 != LoadFloatBigEndian(payload))
        {
            AnalogueIns[inputIndex].CalibrationVolt3 = LoadFloatBigEndian(payload);
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 10:
        if (AnalogueIns[inputIndex].CalibrationValue3 != LoadFloatBigEndian(payload))
        {
            AnalogueIns[inputIndex].CalibrationValue3 = LoadFloatBigEndian(payload);
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 11:
        if (AnalogueIns[inputIndex].NTCBeta != LoadFloatBigEndian(payload))
        {
            AnalogueIns[inputIndex].NTCBeta = LoadFloatBigEndian(payload);
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    case 12:
        if (AnalogueIns[inputIndex].NTCNominalResistance != LoadFloatBigEndian(payload))
        {
            AnalogueIns[inputIndex].NTCNominalResistance = LoadFloatBigEndian(payload);
            configChanged = true;
        }
        if (inputConfigChanged != nullptr)
        {
            *inputConfigChanged = true;
        }
        break;
    default:
        return false;
    }

    if (parameter > 1)
    {
        SanitizeAnalogueInputConfig(AnalogueIns[inputIndex]);
    }

    if (configChanged)
    {
        pendingEEPROMSave = true;
    }

    return true;
}

static bool ApplyExtendedSystemParameter(uint8_t parameter, const uint8_t *payload)
{
    if (payload == nullptr)
    {
        return false;
    }

    bool configChanged = false;

    switch (parameter)
    {
    case 0:
    {
        uint8_t canResEnabled = payload[0] ? 1 : 0;
        if (SystemParams.CANResEnabled != canResEnabled)
        {
            SystemParams.CANResEnabled = canResEnabled;
            digitalWrite(CAN_BUS_RESISTOR_ENABLE, SystemParams.CANResEnabled ? HIGH : LOW);
            configChanged = true;
        }
        break;
    }
    case 1:
    {
        uint16_t canId = LoadUint16BigEndian(payload);
        if (SystemParams.ChannelDataCANID != canId)
        {
            SystemParams.ChannelDataCANID = canId;
            configChanged = true;
        }
        break;
    }
    case 2:
    {
        uint16_t canId = LoadUint16BigEndian(payload);
        if (SystemParams.DigitalInputDataCANID != canId)
        {
            SystemParams.DigitalInputDataCANID = canId;
            configChanged = true;
        }
        break;
    }
    case 3:
    {
        uint16_t canId = LoadUint16BigEndian(payload);
        if (SystemParams.AnalogueInputDataCANID != canId)
        {
            SystemParams.AnalogueInputDataCANID = canId;
            configChanged = true;
        }
        break;
    }
    case 4:
    {
        uint16_t canId = LoadUint16BigEndian(payload);
        if (SystemParams.SystemDataCANID != canId)
        {
            SystemParams.SystemDataCANID = canId;
            configChanged = true;
        }
        break;
    }
    case 5:
    {
        uint16_t canId = LoadUint16BigEndian(payload);
        if (SystemParams.ChannelConfigDataCANID != canId)
        {
            SystemParams.ChannelConfigDataCANID = canId;
            configChanged = true;
        }
        break;
    }
    case 6:
    {
        uint32_t imuWakeWindow = LoadUint32BigEndian(payload);
        if (SystemParams.IMUwakeWindow != imuWakeWindow)
        {
            SystemParams.IMUwakeWindow = imuWakeWindow;
            configChanged = true;
        }
        break;
    }
    case 7:
        if (SystemParams.SpeedUnitPref != payload[0])
        {
            SystemParams.SpeedUnitPref = payload[0];
            configChanged = true;
        }
        break;
    case 8:
        if (SystemParams.DistanceUnitPref != payload[0])
        {
            SystemParams.DistanceUnitPref = payload[0];
            configChanged = true;
        }
        break;
    case 9:
        if (SystemParams.AllowData != payload[0])
        {
            SystemParams.AllowData = payload[0];
            configChanged = true;
        }
        break;
    case 10:
        if (SystemParams.AllowGPS != payload[0])
        {
            SystemParams.AllowGPS = payload[0];
            configChanged = true;
        }
        break;
    case 11:
        if (SystemParams.AllowMotionDetect != payload[0])
        {
            SystemParams.AllowMotionDetect = payload[0];
            configChanged = true;
        }
        break;
    case 12:
    {
        uint16_t canId = LoadUint16BigEndian(payload);
        if (SystemParams.SystemConfigDataCANID != canId)
        {
            SystemParams.SystemConfigDataCANID = canId;
            configChanged = true;
        }
        break;
    }
    case 13:
    {
        TimeZoneRule newRule = {0};
        memcpy(&newRule, payload, sizeof(newRule));
        SanitizeTimeZoneRule(&newRule);

        if (memcmp(&SystemParams.TimeZone, &newRule, sizeof(newRule)) != 0)
        {
            memcpy(&SystemParams.TimeZone, &newRule, sizeof(newRule));
            if (SystemParams.TimeZone.DSTEnabled == 0)
            {
                SystemParams.DSTActive = 0;
            }
            configChanged = true;
        }
        break;
    }
    case 14:
    {
        uint32_t canBusBitrate = LoadUint32BigEndian(payload);
        if (!IsSupportedCANBusBitrate(canBusBitrate))
        {
            return false;
        }

        if (SystemParams.CANBusBitrate != canBusBitrate)
        {
            SystemParams.CANBusBitrate = canBusBitrate;
            pendingCANBitrateReinitialiseOnSave = true;
            configChanged = true;
        }
        break;
    }
    default:
        return false;
    }

    if (configChanged)
    {
        pendingEEPROMSave = true;
    }

    return true;
}

static int32_t ScaleSignedThousandths(float value)
{
    const float maxMagnitude = 2147483.0f;
    if (value > maxMagnitude)
    {
        value = maxMagnitude;
    }
    else if (value < -maxMagnitude)
    {
        value = -maxMagnitude;
    }

    float scaled = value * 1000.0f;
    return (int32_t)(scaled >= 0.0f ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void ReplyWithAnalogueInputStatus(uint8_t inputIndex)
{
    if (inputIndex >= NUM_ANA_CHANNELS)
    {
        return;
    }

    SanitizeAnalogueInputConfig(AnalogueIns[inputIndex]);
    float convertedValue = ReadAnalogueInputValue(inputIndex);
    bool highState = AnalogueIns[inputIndex].InputVoltage >= ANALOG_DIGITAL_THRESHOLD_VOLTS;
    bool digitalActive = AnalogueIns[inputIndex].PullUpEnable ? !highState : highState;
    float millivolts = AnalogueIns[inputIndex].InputVoltage * 1000.0f;
    if (millivolts < 0.0f)
    {
        millivolts = 0.0f;
    }
    if (millivolts > 65535.0f)
    {
        millivolts = 65535.0f;
    }
    uint16_t voltageMilliVolts = (uint16_t)(millivolts + 0.5f);

    CAN_message_t frame0;
    frame0.id = SystemParams.AnalogueInputDataCANID + 1;
    frame0.len = 8;
    frame0.flags.extended = 0;
    frame0.flags.remote = 0;
    frame0.buf[0] = 0;
    frame0.buf[1] = (uint8_t)AnalogueIns[inputIndex].ChanType;
    frame0.buf[2] = (uint8_t)AnalogueIns[inputIndex].Units;
    frame0.buf[3] = (AnalogueIns[inputIndex].PullUpEnable ? 0x01 : 0x00) |
                    (AnalogueIns[inputIndex].PullDownEnable ? 0x02 : 0x00) |
                    (digitalActive ? 0x04 : 0x00);
    StoreUint16BigEndian(&frame0.buf[4], voltageMilliVolts);
    frame0.buf[6] = 0;
    frame0.buf[7] = 0;
    Can.write(frame0);

    CAN_message_t frame1;
    frame1.id = SystemParams.AnalogueInputDataCANID + 1;
    frame1.len = 8;
    frame1.flags.extended = 0;
    frame1.flags.remote = 0;
    frame1.buf[0] = 1;
    StoreInt32BigEndian(&frame1.buf[1], ScaleSignedThousandths(convertedValue));
    frame1.buf[5] = 0;
    frame1.buf[6] = 0;
    frame1.buf[7] = 0;
    Can.write(frame1);
}

uint8_t aliveCounter = 0;

uint32_t EEPROMSaveTimout = 0;

bool pendingEEPROMSave = false;

bool saveEEPROMOnTimeout = false;

void InitialiseCAN()
{
    pinMode(CAN_BUS_RESISTOR_ENABLE, OUTPUT);
    if (SystemParams.CANResEnabled)
    {
        digitalWrite(CAN_BUS_RESISTOR_ENABLE, HIGH);
    }
    else
    {
        digitalWrite(CAN_BUS_RESISTOR_ENABLE, LOW);
    }

    // Initialize CAN bus
    Can.begin();

    // Set baud rate after begin()
    uint32_t canBusBitrate = IsSupportedCANBusBitrate(SystemParams.CANBusBitrate)
                                 ? SystemParams.CANBusBitrate
                                 : DEFAULT_CAN_BUS_BITRATE;
    Can.setBaudRate(canBusBitrate);

    // Accept all messages by default (bank 0 with mask 0 = accept all)
    Can.setFilterSingleMask(0, 0x000, 0x000, STD);
}

void SleepCAN()
{
    pinMode(CAN_BUS_RESISTOR_ENABLE, OUTPUT);
    digitalWrite(CAN_BUS_RESISTOR_ENABLE, LOW);
}

void ReadCANMessages()
{
    CAN_message_t msg;
    bool inputConfigChanged = false;
    while (Can.read(msg))
    {
        if (msg.id == SystemParams.DigitalInputDataCANID)
        {
            CAN_message_t response;
            response.id = SystemParams.DigitalInputDataCANID + 1;
            response.len = 8;
            response.flags.extended = 0;
            response.flags.remote = 0;
            response.buf[0] = BuildDigitalInputMask();
            response.buf[1] = 0;
            response.buf[2] = 0;
            response.buf[3] = 0;
            response.buf[4] = 0;
            response.buf[5] = 0;
            response.buf[6] = 0;
            response.buf[7] = 0;
            Can.write(response);
        }

        if (msg.id == SystemParams.AnalogueInputDataCANID)
        {
            if (msg.len > 0)
            {
                uint8_t requestedInput = msg.buf[0];
                if (requestedInput >= 1 && requestedInput <= NUM_ANA_CHANNELS)
                {
                    ReplyWithAnalogueInputStatus(requestedInput - 1);
                }
            }
        }

        if (msg.id == SystemParams.ChannelDataCANID)
        {
            // Channel status request. Reply on data CAN ID + 1 over 4 frames.
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    CAN_message_t frame0;
                    frame0.id = SystemParams.ChannelDataCANID + 1;
                    frame0.len = 8;
                    frame0.flags.extended = 0;
                    frame0.flags.remote = 0;
                    frame0.buf[0] = 0; // Frame index
                    frame0.buf[1] = (uint8_t)(Channels[i].ChanType);
                    frame0.buf[2] = (ChannelRuntime[i].CurrentValue * 10);
                    frame0.buf[3] = IsChannelRuntimeEnabled(i) ? 1 : 0;
                    uint16_t packedName = ((Channels[i].ChannelName[0] - 'A') << 10) | ((Channels[i].ChannelName[1] - 'A') << 5) | (Channels[i].ChannelName[2] - 'A');
                    frame0.buf[4] = (packedName >> 8) & 0xFF; // upper 8 bits
                    frame0.buf[5] = packedName & 0xFF;        // lower 8 bits
                    frame0.buf[6] = (Channels[i].CurrentThresholdLow * 10);
                    frame0.buf[7] = (Channels[i].CurrentThresholdHigh * 10);
                    Can.write(frame0);

                    CAN_message_t frame1;
                    frame1.id = SystemParams.ChannelDataCANID + 1;
                    frame1.len = 8;
                    frame1.flags.extended = 0;
                    frame1.flags.remote = 0;
                    frame1.buf[0] = 1; // Frame index
                    frame1.buf[1] = Channels[i].RetryCount;
                    frame1.buf[2] = Channels[i].InrushDelay >> 24 & 0xFF; // MSB
                    frame1.buf[3] = Channels[i].InrushDelay >> 16 & 0xFF;
                    frame1.buf[4] = Channels[i].InrushDelay >> 8 & 0xFF;
                    frame1.buf[5] = Channels[i].InrushDelay & 0xFF; // LSB
                    frame1.buf[6] = Channels[i].ActiveHigh;
                    frame1.buf[7] = Channels[i].DelayedOn;
                    Can.write(frame1);

                    CAN_message_t frame2;
                    frame2.id = SystemParams.ChannelDataCANID + 1;
                    frame2.len = 8;
                    frame2.flags.extended = 0;
                    frame2.flags.remote = 0;
                    frame2.buf[0] = 2; // Frame index
                    StoreUint32BigEndian(&frame2.buf[1], Channels[i].DelayedOnTime);
                    frame2.buf[5] = (uint8_t)Channels[i].InrushCurrentThreshold;
                    frame2.buf[6] = Channels[i].DelayedOff;
                    frame2.buf[7] = Channels[i].DelayedOffTrigger;
                    Can.write(frame2);

                    CAN_message_t frame3;
                    frame3.id = SystemParams.ChannelDataCANID + 1;
                    frame3.len = 8;
                    frame3.flags.extended = 0;
                    frame3.flags.remote = 0;
                    frame3.buf[0] = 3; // Frame index
                    StoreUint32BigEndian(&frame3.buf[1], Channels[i].DelayedOffTime);
                    frame3.buf[5] = 0;
                    frame3.buf[6] = 0;
                    frame3.buf[7] = 0;
                    Can.write(frame3);
                }
            }
        }

        // Basic channel control message - F0
        if (msg.id == SystemParams.ChannelConfigDataCANID)
        {
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    CANChannelEnableFlags[i] = (msg.buf[1] > 0) ? true : false;

                    if (Channels[i].ChanType == CAN_PWM)
                    {
                        if (msg.buf[1] <= 100)
                        {
                            Channels[i].PWMSetDuty = msg.buf[1];
                        }
                    }
                }
            }
        }

        // Channel config - F1
        if (msg.id == SystemParams.ChannelConfigDataCANID + 1)
        {
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    // Dertermines which parameters should be set by this message
                    uint8_t paramMask = msg.buf[7];
                    // Channel type
                    if ((paramMask) & 0x01)
                    {
                        if (msg.buf[1] <= 6)
                        {
                            ChannelType newType;
                            switch (msg.buf[1])
                            {
                            case 0: // Digital
                                newType = DIG;
                                break;

                            case 1: // Digital PWM
                                newType = DIG_PWM;
                                break;

                            case 2: // Analogue threshold
                                newType = ANA;
                                break;

                            case 3: // Analogue scaled
                                newType = ANA_PWM;
                                break;

                            case 4: // CAN digital
                                newType = CAN_DIGITAL;
                                break;

                            case 5: // CAN PWM
                                newType = CAN_PWM;
                                break;

                            case 6: // Digital intermittent
                                newType = DIG_INTERMITTENT;
                                break;
                            }

                            if (newType != Channels[i].ChanType)
                            {
                                if (Channels[i].ChanType != DIG_INTERMITTENT && newType == DIG_INTERMITTENT)
                                {
                                    Channels[i].IntermittentOnTime = 1000;
                                    Channels[i].IntermittentOffTime = 1000;
                                }
                                Channels[i].ChanType = newType;
                                pendingEEPROMSave = true;
                                inputConfigChanged = true;
                            }
                        }
                    }

                    // Channel name
                    if ((paramMask >> 1) & 0x01)
                    {
                        uint16_t packedName = ((uint16_t)msg.buf[2] << 8) | msg.buf[3];
                        char newName[4];
                        newName[0] = ((packedName >> 10) & 0x1F) + 'A';
                        newName[1] = ((packedName >> 5) & 0x1F) + 'A';
                        newName[2] = (packedName & 0x1F) + 'A';
                        newName[3] = 0;

                        if (memcmp(newName, Channels[i].ChannelName, 3) != 0)
                        {
                            memcpy(Channels[i].ChannelName, newName, sizeof(Channels[i].ChannelName));
                            pendingEEPROMSave = true;
                        }
                    }

                    // Current threshold low
                    if ((paramMask >> 3) & 0x01)
                    {
                        if (msg.buf[4] <= MAX_CURRENT_X10)
                        {
                            uint8_t newThreshold = msg.buf[4] / 10;
                            if (newThreshold != Channels[i].CurrentThresholdLow)
                            {
                                Channels[i].CurrentThresholdLow = newThreshold;
                                pendingEEPROMSave = true;
                            }
                        }
                    }

                    // Current threshold high
                    if ((paramMask >> 4) & 0x01)
                    {
                        if (msg.buf[5] <= MAX_CURRENT_X10)
                        {
                            uint8_t newThreshold = msg.buf[5] / 10;
                            if (newThreshold != Channels[i].CurrentThresholdHigh)
                            {
                                Channels[i].CurrentThresholdHigh = newThreshold;
                                pendingEEPROMSave = true;
                            }
                        }
                    }

                    // Retry count
                    if ((paramMask >> 5) & 0x01)
                    {
                        if (msg.buf[6] != Channels[i].RetryCount)
                        {
                            Channels[i].RetryCount = msg.buf[6];
                            pendingEEPROMSave = true;
                        }
                    }
                }
            }
        }

        // Channel config - F2
        if (msg.id == SystemParams.ChannelConfigDataCANID + 2)
        {
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    // Dertermines which parameters should be set by this message
                    uint8_t paramMask = msg.buf[7];

                    // Inrush delay
                    if ((paramMask >> 1) & 0x01)
                    {
                        uint32_t inrush = ((uint32_t)msg.buf[1] << 24) |
                                          ((uint32_t)msg.buf[2] << 16) |
                                          ((uint32_t)msg.buf[3] << 8) |
                                          (uint32_t)msg.buf[4];

                        if (inrush <= MAX_INRUSH_DELAY && inrush != Channels[i].InrushDelay)
                        {
                            Channels[i].InrushDelay = inrush;
                            pendingEEPROMSave = true;
                        }
                    }

                    // Active high
                    if ((paramMask >> 2) & 0x01)
                    {
                        bool newActiveHigh = (msg.buf[5] != 0);
                        if (newActiveHigh != Channels[i].ActiveHigh)
                        {
                            Channels[i].ActiveHigh = newActiveHigh;
                            pendingEEPROMSave = true;
                        }
                    }

                }
            }
        }

        // Channel config - F3
        if (msg.id == SystemParams.ChannelConfigDataCANID + 3)
        {
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    // Dertermines which parameters should be set by this message
                    uint8_t paramMask = msg.buf[7];

                    // Soft start enable
                    if ((paramMask >> 2) & 0x01)
                    {
                        bool newSoftStart = (msg.buf[5] != 0);
                        if (newSoftStart != Channels[i].SoftStart)
                        {
                            Channels[i].SoftStart = newSoftStart;
                            pendingEEPROMSave = true;
                        }
                    }

                    // Soft start time
                    if ((paramMask >> 3) & 0x01)
                    {
                        uint32_t softStartTime = (msg.buf[6] * 1000); // Convert seconds to milliseconds

                        if (softStartTime <= MAX_SOFT_START_TIME && softStartTime != Channels[i].SoftStartTime)
                        {
                            Channels[i].SoftStartTime = softStartTime;
                            pendingEEPROMSave = true;
                        }
                    }
                }
            }
        }

        // Channel config - F4
        if (msg.id == SystemParams.ChannelConfigDataCANID + 4)
        {
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    uint8_t paramMask = msg.buf[7];

                    if ((paramMask & 0x01) && msg.buf[1] <= INRUSH_CURRENT_MAX)
                    {
                        float newInrushCurrentThreshold = (float)msg.buf[1];
                        if (newInrushCurrentThreshold != Channels[i].InrushCurrentThreshold)
                        {
                            Channels[i].InrushCurrentThreshold = newInrushCurrentThreshold;
                            pendingEEPROMSave = true;
                        }
                    }
                }
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 5)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS)
            {
                uint8_t channelIndex = msg.buf[0] - 1;
                uint8_t paramMask = msg.buf[7];

                if ((paramMask >> 4) & 0x01)
                {
                    ApplyExtendedChannelParameter(channelIndex, 16, &msg.buf[5], &inputConfigChanged);
                }
                if ((paramMask >> 5) & 0x01)
                {
                    ApplyExtendedChannelParameter(channelIndex, 25, &msg.buf[6], &inputConfigChanged);
                }
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 6)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS)
            {
                uint8_t channelIndex = msg.buf[0] - 1;
                uint8_t paramMask = msg.buf[7];

                if (paramMask & 0x01)
                {
                    ApplyExtendedChannelParameter(channelIndex, 17, &msg.buf[1], &inputConfigChanged);
                }
                if ((paramMask >> 1) & 0x01)
                {
                    ApplyExtendedChannelParameter(channelIndex, 21, &msg.buf[5], &inputConfigChanged);
                }
                if ((paramMask >> 2) & 0x01)
                {
                    ApplyExtendedChannelParameter(channelIndex, 22, &msg.buf[6], &inputConfigChanged);
                }
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 7)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS)
            {
                uint8_t channelIndex = msg.buf[0] - 1;
                uint8_t paramMask = msg.buf[7];

                if (paramMask & 0x01)
                {
                    ApplyExtendedChannelParameter(channelIndex, 18, &msg.buf[1], &inputConfigChanged);
                }
                if ((paramMask >> 1) & 0x01)
                {
                    ApplyExtendedChannelParameter(channelIndex, 23, &msg.buf[5], &inputConfigChanged);
                }
                if ((paramMask >> 2) & 0x01)
                {
                    uint8_t delayedOn = (msg.buf[6] & 0x01) ? 1 : 0;
                    ApplyExtendedChannelParameter(channelIndex, 28, &delayedOn, &inputConfigChanged);
                }
                if ((paramMask >> 3) & 0x01)
                {
                    uint8_t delayedOff = (msg.buf[6] & 0x02) ? 1 : 0;
                    ApplyExtendedChannelParameter(channelIndex, 30, &delayedOff, &inputConfigChanged);
                }
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 8)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS)
            {
                uint8_t channelIndex = msg.buf[0] - 1;
                uint8_t paramMask = msg.buf[7];

                if (paramMask & 0x01)
                {
                    ApplyExtendedChannelParameter(channelIndex, 19, &msg.buf[1], &inputConfigChanged);
                }
                if ((paramMask >> 1) & 0x01)
                {
                    ApplyExtendedChannelParameter(channelIndex, 32, &msg.buf[5], &inputConfigChanged);
                }
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 9)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedChannelParameter(msg.buf[0] - 1, 20, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 10)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedChannelParameter(msg.buf[0] - 1, 24, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 11)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedChannelParameter(msg.buf[0] - 1, 26, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 12)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedChannelParameter(msg.buf[0] - 1, 27, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 13)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedChannelParameter(msg.buf[0] - 1, 29, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 14)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedChannelParameter(msg.buf[0] - 1, 31, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 15)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_ANA_CHANNELS)
            {
                uint8_t inputIndex = msg.buf[0] - 1;
                uint8_t paramMask = msg.buf[7];

                if (paramMask & 0x01)
                {
                    ApplyExtendedAnalogueParameter(inputIndex, 0, &msg.buf[1], &inputConfigChanged);
                }
                if ((paramMask >> 1) & 0x01)
                {
                    ApplyExtendedAnalogueParameter(inputIndex, 1, &msg.buf[2], &inputConfigChanged);
                }
                if ((paramMask >> 2) & 0x01)
                {
                    ApplyExtendedAnalogueParameter(inputIndex, 2, &msg.buf[3], &inputConfigChanged);
                }
                if ((paramMask >> 3) & 0x01)
                {
                    ApplyExtendedAnalogueParameter(inputIndex, 3, &msg.buf[4], &inputConfigChanged);
                }
                if ((paramMask >> 4) & 0x01)
                {
                    ApplyExtendedAnalogueParameter(inputIndex, 4, &msg.buf[5], &inputConfigChanged);
                }
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 16)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_ANA_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedAnalogueParameter(msg.buf[0] - 1, 5, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 17)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_ANA_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedAnalogueParameter(msg.buf[0] - 1, 6, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 18)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_ANA_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedAnalogueParameter(msg.buf[0] - 1, 7, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 19)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_ANA_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedAnalogueParameter(msg.buf[0] - 1, 8, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 20)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_ANA_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedAnalogueParameter(msg.buf[0] - 1, 9, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 21)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_ANA_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedAnalogueParameter(msg.buf[0] - 1, 10, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 22)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_ANA_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedAnalogueParameter(msg.buf[0] - 1, 11, &msg.buf[1], &inputConfigChanged);
            }
        }

        if (msg.id == SystemParams.ChannelConfigDataCANID + 23)
        {
            if (msg.len >= 8 && msg.buf[0] >= 1 && msg.buf[0] <= NUM_ANA_CHANNELS && (msg.buf[7] & 0x01))
            {
                ApplyExtendedAnalogueParameter(msg.buf[0] - 1, 12, &msg.buf[1], &inputConfigChanged);
            }
        }

        // System config message
        if (msg.id == SystemParams.SystemConfigDataCANID)
        {
            // Dertermines which parameters should be set by this message
            uint8_t paramMask = msg.buf[7];

            // System current limit
            if (paramMask & 0x01)
            {
                if (msg.buf[0] <= SYSTEM_CURRENT_MAX && msg.buf[0] != SystemParams.SystemCurrentLimit)
                {
                    SystemParams.SystemCurrentLimit = msg.buf[0];
                    pendingEEPROMSave = true;
                }
            }

            // Speed unit preference
            if ((paramMask >> 1) & 0x01)
            {
                if (msg.buf[1] <= 1 && msg.buf[1] != SystemParams.SpeedUnitPref)
                {
                    SystemParams.SpeedUnitPref = msg.buf[1];
                    pendingEEPROMSave = true;
                }
            }

            // Distance unit preference
            if ((paramMask >> 2) & 0x01)
            {
                if (msg.buf[2] <= 1 && msg.buf[2] != SystemParams.DistanceUnitPref)
                {
                    SystemParams.DistanceUnitPref = msg.buf[2];
                    pendingEEPROMSave = true;
                }
            }

            // Allow data
            if ((paramMask >> 3) & 0x01)
            {
                if (msg.buf[3] <= 1 && msg.buf[3] != SystemParams.AllowData)
                {
                    SystemParams.AllowData = msg.buf[3];
                    pendingEEPROMSave = true;
                }
            }

            // Allow GPS
            if ((paramMask >> 4) & 0x01)
            {
                if (msg.buf[4] <= 1 && msg.buf[4] != SystemParams.AllowGPS)
                {
                    SystemParams.AllowGPS = msg.buf[4];
                    pendingEEPROMSave = true;
                }
            }

            // Allow motion detect
            if ((paramMask >> 5) & 0x01)
            {
                if (msg.buf[5] <= 1 && msg.buf[5] != SystemParams.AllowMotionDetect)
                {
                    SystemParams.AllowMotionDetect = msg.buf[5];
                    pendingEEPROMSave = true;
                }
            }

            // Motion dead time
            if ((paramMask >> 6) & 0x01)
            {
                if (msg.buf[6] <= MAX_MOTION_DEAD_TIME && msg.buf[6] != SystemParams.MotionDeadTime)
                {
                    SystemParams.MotionDeadTime = msg.buf[6];
                    pendingEEPROMSave = true;
                }
            }
        }

        if (msg.id == SystemParams.SystemConfigDataCANID + 1)
        {
            if (msg.len >= 8)
            {
                uint8_t paramMask = msg.buf[7];

                if (paramMask & 0x01)
                {
                    ApplyExtendedSystemParameter(0, &msg.buf[0]);
                }
                if ((paramMask >> 1) & 0x01)
                {
                    ApplyExtendedSystemParameter(1, &msg.buf[1]);
                }
                if ((paramMask >> 2) & 0x01)
                {
                    ApplyExtendedSystemParameter(2, &msg.buf[3]);
                }
                if ((paramMask >> 3) & 0x01)
                {
                    ApplyExtendedSystemParameter(3, &msg.buf[5]);
                }
            }
        }

        if (msg.id == SystemParams.SystemConfigDataCANID + 2)
        {
            if (msg.len >= 8)
            {
                uint8_t paramMask = msg.buf[7];

                if (paramMask & 0x01)
                {
                    ApplyExtendedSystemParameter(4, &msg.buf[0]);
                }
                if ((paramMask >> 1) & 0x01)
                {
                    ApplyExtendedSystemParameter(5, &msg.buf[2]);
                }
                if ((paramMask >> 2) & 0x01)
                {
                    ApplyExtendedSystemParameter(12, &msg.buf[4]);
                }
            }
        }

        if (msg.id == SystemParams.SystemConfigDataCANID + 3)
        {
            if (msg.len >= 8)
            {
                uint8_t paramMask = msg.buf[7];

                if (paramMask & 0x01)
                {
                    ApplyExtendedSystemParameter(6, &msg.buf[0]);
                }
            }
        }

        if (msg.id == SystemParams.SystemConfigDataCANID + 4)
        {
            if (msg.len >= 8)
            {
                uint8_t paramMask = msg.buf[7];

                if (paramMask & 0x01)
                {
                    ApplyExtendedSystemParameter(14, &msg.buf[0]);
                }
            }
        }

        if (msg.id == SystemParams.SystemConfigDataCANID + 5)
        {
            if (msg.len == 8)
            {
                memcpy(&pendingTimeZoneConfigBytes[0], msg.buf, 8);
                pendingTimeZoneConfigMask |= 0x01;
            }
        }

        if (msg.id == SystemParams.SystemConfigDataCANID + 6)
        {
            if (msg.len >= 7)
            {
                memcpy(&pendingTimeZoneConfigBytes[8], msg.buf, sizeof(TimeZoneRule) - 8);
                pendingTimeZoneConfigMask |= 0x02;
            }

            if (pendingTimeZoneConfigMask == 0x03)
            {
                ApplyExtendedSystemParameter(13, pendingTimeZoneConfigBytes);
                pendingTimeZoneConfigMask = 0;
            }
        }

        if (pendingEEPROMSave)
        {
            pendingEEPROMSave = false;
            ScheduleCANConfigSave();
        }
    }

    if (inputConfigChanged)
    {
        InitialiseInputs();
    }
}

void PersistPendingCANConfigChanges()
{
    SaveChannelConfig();
    SaveSystemConfig();
    SaveAnalogueConfig();

    if (pendingCANBitrateReinitialiseOnSave)
    {
        InitialiseCAN();
        pendingCANBitrateReinitialiseOnSave = false;
    }
}

void BroadcastSystemStatus()
{
    // System status 1
    CAN_message_t systemStatusMsg1;
    systemStatusMsg1.id = SystemParams.SystemDataCANID;
    systemStatusMsg1.len = 8;
    systemStatusMsg1.flags.extended = 0;
    systemStatusMsg1.flags.remote = 0;
    systemStatusMsg1.buf[0] = aliveCounter++;
    systemStatusMsg1.buf[1] = SystemParams.SystemCurrentLimit;
    systemStatusMsg1.buf[2] = (uint8_t)SystemRuntimeParams.SystemTemperature;
    systemStatusMsg1.buf[3] = (uint8_t)(SystemRuntimeParams.VBatt * 10);
    uint16_t scaledCurrent = (uint16_t)(SystemRuntimeParams.SystemCurrent * 10);
    systemStatusMsg1.buf[4] = (scaledCurrent >> 8) & 0xFF;                             // MSB
    systemStatusMsg1.buf[5] = scaledCurrent & 0xFF;                                    // LSB
    systemStatusMsg1.buf[6] = (uint8_t)((SystemRuntimeParams.ErrorFlags >> 8) & 0xFF); // MSB
    systemStatusMsg1.buf[7] = (uint8_t)(SystemRuntimeParams.ErrorFlags & 0xFF);        // LSB
    Can.write(systemStatusMsg1);

    // System status 2
    CAN_message_t systemStatusMsg2;
    systemStatusMsg2.id = SystemParams.SystemDataCANID + 1;
    systemStatusMsg2.len = 8;
    systemStatusMsg2.flags.extended = 0;
    systemStatusMsg2.flags.remote = 0;
    systemStatusMsg2.buf[0] = aliveCounter;
    systemStatusMsg2.buf[1] = SystemParams.SpeedUnitPref;
    systemStatusMsg2.buf[2] = SystemParams.DistanceUnitPref;
    systemStatusMsg2.buf[3] = SystemParams.AllowData;
    systemStatusMsg2.buf[4] = SystemParams.AllowGPS;
    systemStatusMsg2.buf[5] = SystemParams.AllowMotionDetect;
    systemStatusMsg2.buf[6] = SystemParams.MotionDeadTime;
    systemStatusMsg2.buf[7] = 0;
    Can.write(systemStatusMsg2);
}
