/*  InputHandler.cpp Input handler deals with digital channel input status.
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

#include "InputHandler.h"
#include <math.h>

static bool intermittentInputActive[NUM_CHANNELS] = {false};
static bool intermittentOutputActive[NUM_CHANNELS] = {false};
static uint32_t intermittentPhaseTimers[NUM_CHANNELS] = {0};

static void SetChannelRuntimeEnabledState(uint8_t channelIndex, bool enabled)
{
    if (channelIndex >= NUM_CHANNELS)
    {
        return;
    }

    ChannelRuntime[channelIndex].Enabled = enabled ? 1 : 0;

    if (enabledFlags[channelIndex] != enabled)
    {
        enabledFlags[channelIndex] = enabled;
        if (enabled)
        {
            enabledTimers[channelIndex] = millis();
        }
    }
}

float CalculateLinear(float x, float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    if (fabsf(dx) < MIN_DENOMINATOR)
    {
        return y1;
    }
    return y1 + ((x - x1) * (y2 - y1) / dx);
}

float CalculateQuadraticLagrange(float x, float x1, float y1, float x2, float y2, float x3, float y3)
{
    float d1 = (x1 - x2) * (x1 - x3);
    float d2 = (x2 - x1) * (x2 - x3);
    float d3 = (x3 - x1) * (x3 - x2);

    if (fabsf(d1) < MIN_DENOMINATOR || fabsf(d2) < MIN_DENOMINATOR || fabsf(d3) < MIN_DENOMINATOR)
    {
        return CalculateLinear(x, x1, y1, x2, y2);
    }

    float l1 = ((x - x2) * (x - x3)) / d1;
    float l2 = ((x - x1) * (x - x3)) / d2;
    float l3 = ((x - x1) * (x - x2)) / d3;
    return (l1 * y1) + (l2 * y2) + (l3 * y3);
}

bool IsTemperatureUnits(uint8_t units)
{
    return units == ANA_UNITS_CELSIUS || units == ANA_UNITS_FAHRENHEIT;
}

void SanitizeAnalogueInputConfig(AnalogueInputs &input)
{
    if (input.CalibrationPoints < 2)
    {
        input.CalibrationPoints = 2;
    }
    if (input.CalibrationPoints > 3)
    {
        input.CalibrationPoints = 3;
    }

    if (input.NTCBeta < 1.0f)
    {
        input.NTCBeta = 3950.0f;
    }
    if (input.NTCNominalResistance < 1.0f)
    {
        input.NTCNominalResistance = 10000.0f;
    }

    switch (input.ChanType)
    {
    case RAW_VOLTAGE:
        input.PullUpEnable = false;
        input.PullDownEnable = false;
        input.Units = ANA_UNITS_VOLTS;
        break;
    case ACTIVE:
        input.PullUpEnable = false;
        input.PullDownEnable = false;
        break;
    case PASSIVE:
        if (!input.PullUpEnable && !input.PullDownEnable)
        {
            input.PullUpEnable = true;
        }
        if (input.PullUpEnable && input.PullDownEnable)
        {
            input.PullDownEnable = false;
        }
        break;
    case NTC:
        if (!input.PullUpEnable && !input.PullDownEnable)
        {
            input.PullUpEnable = true;
        }
        if (input.PullUpEnable && input.PullDownEnable)
        {
            input.PullDownEnable = false;
        }
        if (!IsTemperatureUnits(input.Units))
        {
            input.Units = ANA_UNITS_CELSIUS;
        }
        break;
    case DIGITAL:
        if (!input.PullUpEnable && !input.PullDownEnable)
        {
            input.PullDownEnable = true;
        }
        if (input.PullUpEnable && input.PullDownEnable)
        {
            input.PullDownEnable = false;
        }
        input.Units = ANA_UNITS_VOLTS;
        break;
    default:
        input.ChanType = RAW_VOLTAGE;
        input.PullUpEnable = false;
        input.PullDownEnable = false;
        input.Units = ANA_UNITS_VOLTS;
        break;
    }
}

float ConvertUsingCalibration(const AnalogueInputs &input, float sourceVoltage)
{
    if (input.CalibrationPoints >= 3)
    {
        return CalculateQuadraticLagrange(
            sourceVoltage,
            input.CalibrationVolt1, input.CalibrationValue1,
            input.CalibrationVolt2, input.CalibrationValue2,
            input.CalibrationVolt3, input.CalibrationValue3);
    }

    return CalculateLinear(
        sourceVoltage,
        input.CalibrationVolt1,
        input.CalibrationValue1,
        input.CalibrationVolt2,
        input.CalibrationValue2);
}

float ConvertUsingNTC(const AnalogueInputs &input, float sourceVoltage)
{
    float voltage = sourceVoltage;
    if (voltage < 0.001f)
    {
        voltage = 0.001f;
    }
    if (voltage > (ANALOG_SENSOR_SUPPLY_VOLTAGE - 0.001f))
    {
        voltage = ANALOG_SENSOR_SUPPLY_VOLTAGE - 0.001f;
    }

    float resistance = input.NTCNominalResistance;
    if (input.PullUpEnable)
    {
        resistance = ANALOG_PULL_RESISTOR_OHMS * voltage / (ANALOG_SENSOR_SUPPLY_VOLTAGE - voltage);
    }
    else if (input.PullDownEnable)
    {
        resistance = ANALOG_PULL_RESISTOR_OHMS * (ANALOG_SENSOR_SUPPLY_VOLTAGE - voltage) / voltage;
    }

    if (resistance < 1.0f)
    {
        resistance = 1.0f;
    }

    float invT = (1.0f / NTC_T0_KELVIN) + (logf(resistance / input.NTCNominalResistance) / input.NTCBeta);
    if (fabsf(invT) < MIN_DENOMINATOR)
    {
        return 0.0f;
    }

    float tempC = (1.0f / invT) - 273.15f;
    if (input.Units == ANA_UNITS_FAHRENHEIT)
    {
        return (tempC * 1.8f) + 32.0f;
    }

    return tempC;
}

float ReadAnalogueInputValue(int inputIndex)
{
    float sourceVoltage = analogRead(AnalogueIns[inputIndex].InputPin) * (V_REF / ADCres);
    sourceVoltage *= ANALOG_INPUT_PIN_TO_SOURCE_GAIN;
    AnalogueIns[inputIndex].InputVoltage = sourceVoltage;

    float converted = sourceVoltage;
    switch (AnalogueIns[inputIndex].ChanType)
    {
    case RAW_VOLTAGE:
        converted = sourceVoltage;
        break;
    case ACTIVE:
    case PASSIVE:
        converted = ConvertUsingCalibration(AnalogueIns[inputIndex], sourceVoltage);
        break;
    case NTC:
        converted = ConvertUsingNTC(AnalogueIns[inputIndex], sourceVoltage);
        break;
    case DIGITAL:
        converted = sourceVoltage;
        break;
    default:
        converted = sourceVoltage;
        break;
    }

    AnalogueIns[inputIndex].InputValue = converted;
    return converted;
}

bool IsChannelRuntimeEnabled(uint8_t channelIndex)
{
    return (channelIndex < NUM_CHANNELS) && (ChannelRuntime[channelIndex].Enabled != 0);
}

static bool ReadAnalogueInputHighState(int inputIndex)
{
    float sourceVoltage = ReadAnalogueInputValue(inputIndex);
    return sourceVoltage >= ANALOG_DIGITAL_THRESHOLD_VOLTS;
}

bool ReadAnalogueInputAsDigital(int inputIndex)
{
    bool high = ReadAnalogueInputHighState(inputIndex);

    // Pull-up means active low at the connector; pull-down means active high.
    if (AnalogueIns[inputIndex].PullUpEnable)
    {
        return !high;
    }

    return high;
}

bool SyncChannelTypeForAssignedInput(uint8_t channelIndex)
{
    if (channelIndex >= NUM_CHANNELS)
    {
        return false;
    }

    int inputIndex = -1;
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        if (Channels[channelIndex].InputControlPin == ANAchannelInputPins[i])
        {
            inputIndex = i;
            break;
        }
    }

    if (inputIndex < 0)
    {
        return false;
    }

    if (AnalogueIns[inputIndex].ChanType == DIGITAL)
    {
        if (Channels[channelIndex].ChanType == ANA)
        {
            Channels[channelIndex].ChanType = DIG;
            return true;
        }
        if (Channels[channelIndex].ChanType == ANA_PWM)
        {
            Channels[channelIndex].ChanType = DIG_PWM;
            return true;
        }
        return false;
    }

    if (Channels[channelIndex].ChanType == DIG)
    {
        Channels[channelIndex].ChanType = ANA;
        return true;
    }
    if (Channels[channelIndex].ChanType == DIG_PWM)
    {
        Channels[channelIndex].ChanType = ANA_PWM;
        return true;
    }

    return false;
}

bool SyncChannelTypesForAnalogueInput(uint8_t inputIndex)
{
    if (inputIndex >= NUM_ANA_CHANNELS)
    {
        return false;
    }

    bool changedAny = false;
    for (int channelIndex = 0; channelIndex < NUM_CHANNELS; channelIndex++)
    {
        if (Channels[channelIndex].InputControlPin != ANAchannelInputPins[inputIndex])
        {
            continue;
        }

        if (SyncChannelTypeForAssignedInput(channelIndex))
        {
            changedAny = true;
        }
    }

    return changedAny;
}

static void ResetIntermittentState(uint8_t channelIndex)
{
    intermittentInputActive[channelIndex] = false;
    intermittentOutputActive[channelIndex] = false;
    intermittentPhaseTimers[channelIndex] = 0;
}

static void ResolveChannelInputSource(uint8_t channelIndex, int *inputPin, bool *inputIsDigital)
{
    if (inputPin == nullptr || inputIsDigital == nullptr)
    {
        return;
    }

    *inputPin = -1;
    *inputIsDigital = false;

    if (channelIndex >= NUM_CHANNELS)
    {
        return;
    }

    if (Channels[channelIndex].InputControlPin == IGN_INPUT)
    {
        *inputIsDigital = true;
        return;
    }

    for (int i = 0; i < NUM_DI_CHANNELS; i++)
    {
        if (Channels[channelIndex].InputControlPin == DIchannelInputPins[i])
        {
            *inputPin = i;
            *inputIsDigital = true;
            return;
        }
    }

    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        if (Channels[channelIndex].InputControlPin == ANAchannelInputPins[i])
        {
            *inputPin = i;
            return;
        }
    }
}

static bool ReadChannelDigitalInputState(uint8_t channelIndex, int inputPin, bool inputIsDigital, bool requireDigitalAnalogueConfig)
{
    extern volatile uint8_t PowerState;
    extern bool runOnEligible[NUM_CHANNELS];
    extern uint32_t runOnDeadline[NUM_CHANNELS];

    if ((PowerState == 1 /*PREPARE_SLEEP*/ || PowerState == 2 /*SLEEPING*/) &&
        Channels[channelIndex].RunOn &&
        runOnEligible[channelIndex] &&
        runOnDeadline[channelIndex] != 0 &&
        (int32_t)(millis() - runOnDeadline[channelIndex]) < 0)
    {
        return true;
    }

    if (ChannelRuntime[channelIndex].Override)
    {
        return true;
    }

    if (inputIsDigital)
    {
        return digitalRead(Channels[channelIndex].InputControlPin);
    }

    if (inputPin < 0)
    {
        return false;
    }

    if (requireDigitalAnalogueConfig && AnalogueIns[inputPin].ChanType != DIGITAL)
    {
        return false;
    }

    return ReadAnalogueInputAsDigital(inputPin);
}

static bool IsChannelOverrideActive(uint8_t channelIndex)
{
    return (channelIndex < NUM_CHANNELS) && (ChannelRuntime[channelIndex].Override != 0);
}

static bool ComputeIntermittentOutputState(uint8_t channelIndex, bool inputActive)
{
    uint32_t now = millis();
    uint32_t onTime = Channels[channelIndex].IntermittentOnTime;
    uint32_t offTime = Channels[channelIndex].IntermittentOffTime;

    if (!inputActive)
    {
        ResetIntermittentState(channelIndex);
        return false;
    }

    if (!intermittentInputActive[channelIndex])
    {
        intermittentInputActive[channelIndex] = true;
        intermittentOutputActive[channelIndex] = true;
        intermittentPhaseTimers[channelIndex] = now;
    }

    if (onTime == 0 && offTime == 0)
    {
        intermittentOutputActive[channelIndex] = true;
        return true;
    }

    if (onTime == 0)
    {
        intermittentOutputActive[channelIndex] = false;
        return false;
    }

    if (offTime == 0)
    {
        intermittentOutputActive[channelIndex] = true;
        return true;
    }

    uint32_t phaseDuration = intermittentOutputActive[channelIndex] ? onTime : offTime;
    if ((now - intermittentPhaseTimers[channelIndex]) >= phaseDuration)
    {
        intermittentOutputActive[channelIndex] = !intermittentOutputActive[channelIndex];
        intermittentPhaseTimers[channelIndex] = now;
    }

    return intermittentOutputActive[channelIndex];
}

void InitialiseInputs()
{
    analogReadResolution(12);

    // Ignition inout is used for wake/sleep
    pinMode(IGN_INPUT, INPUT_PULLDOWN);

    // Digital inputs. Default to active-high
    for (int i = 0; i < NUM_DI_CHANNELS; i++)
    {
        pinMode(DIchannelInputPins[i], INPUT_PULLDOWN);
    }

    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        AnalogueIns[i].InputPin = ANAchannelInputPins[i];
        AnalogueIns[i].PullDownPin = ANAchannelInputPullDowns[i];
        AnalogueIns[i].PullUpPin = ANAchannelInputPullUps[i];
        SanitizeAnalogueInputConfig(AnalogueIns[i]);

        pinMode(AnalogueIns[i].PullDownPin, OUTPUT);
        pinMode(AnalogueIns[i].PullUpPin, OUTPUT);

        digitalWrite(AnalogueIns[i].PullDownPin, AnalogueIns[i].PullDownEnable ? HIGH : LOW);
        digitalWrite(AnalogueIns[i].PullUpPin, AnalogueIns[i].PullUpEnable ? HIGH : LOW);

        // Analogue-capable inputs are always sampled through the ADC path, even when used logically as digital.
        pinMode(AnalogueIns[i].InputPin, INPUT_ANALOG);
    }
}

void HandleInputs()
{
    analogReadResolution(12);

    // Check channel type and enable for active level
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        if (Channels[i].ChanType != DIG_INTERMITTENT)
        {
            ResetIntermittentState(i);
        }

        // Find the input pin index first and what type it is
        int inputPin = -1;
        bool inputIsDigital = false;
        ResolveChannelInputSource(i, &inputPin, &inputIsDigital);
        switch (Channels[i].ChanType)
        {

        case DIG:
        case DIG_PWM:
        {
            bool runtimeEnabled = ReadChannelDigitalInputState(i, inputPin, inputIsDigital, false);
            SetChannelRuntimeEnabledState(i, runtimeEnabled);
            break;
        }
        case DIG_INTERMITTENT:
        {
            bool inputActive = ReadChannelDigitalInputState(i, inputPin, inputIsDigital, true);
            bool runtimeEnabled = ComputeIntermittentOutputState(i, inputActive);
            SetChannelRuntimeEnabledState(i, runtimeEnabled);
            break;
        }
        case ANA:
        {
            if (IsChannelOverrideActive(i))
            {
                SetChannelRuntimeEnabledState(i, true);
                break;
            }

            if (inputPin < 0)
            {
                SetChannelRuntimeEnabledState(i, false);
                break;
            }

            // Threshold-based analogue input
            SanitizeAnalogueInputConfig(AnalogueIns[inputPin]);
            float value = ReadAnalogueInputValue(inputPin);
            bool negativeGoingThreshold = Channels[i].OnThreshold < Channels[i].OffThreshold;
            bool runtimeEnabled = IsChannelRuntimeEnabled(i);
            if ((!negativeGoingThreshold && value >= Channels[i].OnThreshold) ||
                (negativeGoingThreshold && value <= Channels[i].OnThreshold))
            {
                runtimeEnabled = true;
            }
            else if ((!negativeGoingThreshold && value < Channels[i].OffThreshold) ||
                     (negativeGoingThreshold && value > Channels[i].OffThreshold))
            {
                runtimeEnabled = false;
            }
            SetChannelRuntimeEnabledState(i, runtimeEnabled);
            break;
        }
        case ANA_PWM:
        {
            if (IsChannelOverrideActive(i))
            {
                SetChannelRuntimeEnabledState(i, Channels[i].PWMSetDuty > 0);
                break;
            }

            if (inputPin < 0)
            {
                SetChannelRuntimeEnabledState(i, false);
                Channels[i].PWMSetDuty = 0;
                break;
            }

            // Scaled PWM analogue input
            SanitizeAnalogueInputConfig(AnalogueIns[inputPin]);
            float value = ReadAnalogueInputValue(inputPin);
            float scaleMin = Channels[i].ScaleMin;
            float scaleMax = Channels[i].ScaleMax;
            float scaleRange = scaleMax - scaleMin;
            if (fabsf(scaleRange) < MIN_DENOMINATOR)
            {
                scaleRange = (scaleRange < 0.0f) ? -MIN_DENOMINATOR : MIN_DENOMINATOR;
            }
            float norm = (value - scaleMin) / scaleRange;
            if (norm < 0.0f)
            {
                norm = 0.0f;
            }
            if (norm > 1.0f)
            {
                norm = 1.0f;
            }
            uint8_t pwmMin = Channels[i].PWMMin;
            uint8_t pwmMax = Channels[i].PWMMax;
            float dutyFloat = ((float)pwmMin) + (((float)pwmMax - (float)pwmMin) * norm);
            int dutyInt = (int)lroundf(dutyFloat);
            if (dutyInt < 0)
            {
                dutyInt = 0;
            }
            if (dutyInt > 100)
            {
                dutyInt = 100;
            }
            Channels[i].PWMSetDuty = (uint8_t)dutyInt;
            SetChannelRuntimeEnabledState(i, dutyInt > 0);
            break;
        }
        case CAN_DIGITAL:
        case CAN_PWM:
        {
            bool runtimeEnabled = false;
            if (IsChannelOverrideActive(i))
            {
                runtimeEnabled = (Channels[i].ChanType == CAN_PWM) ? (Channels[i].PWMSetDuty > 0) : true;
            }
            else
            {
                runtimeEnabled = CANChannelEnableFlags[i];
            }

            SetChannelRuntimeEnabledState(i, runtimeEnabled);

            if (!runtimeEnabled)
            {
                // Clear error flags on disable
                ChannelRuntime[i].ErrorFlags = 0;
            }
            break;
        }
        default:
            break;
        }

        if (!IsChannelRuntimeEnabled(i))
        {
            // Clear error flags on disable
            ChannelRuntime[i].ErrorFlags = 0;
        }
    }

    // Check analogue inputs. Set pull-ups/pull-downs
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        SanitizeAnalogueInputConfig(AnalogueIns[i]);
        digitalWrite(AnalogueIns[i].PullDownPin, AnalogueIns[i].PullDownEnable);
        digitalWrite(AnalogueIns[i].PullUpPin, AnalogueIns[i].PullUpEnable);
    }
}

void PullResistorSleep()
{
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        pinMode(AnalogueIns[i].PullDownPin, INPUT_ANALOG);
        pinMode(AnalogueIns[i].PullUpPin, INPUT_ANALOG);
    }
}