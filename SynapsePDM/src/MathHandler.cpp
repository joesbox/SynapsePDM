#include "MathHandler.h"
#include "InputHandler.h"
#include "IMU.h"
#include "CustomCAN.h"
#include <math.h>

static bool hysteresisState[NUM_CHANNELS][MAX_MATH_RULES_PER_CHANNEL] = {{false}};
static bool toggleState[NUM_CHANNELS][MAX_MATH_RULES_PER_CHANNEL] = {{false}};
static bool togglePrevInput[NUM_CHANNELS][MAX_MATH_RULES_PER_CHANNEL] = {{false}};
static const uint16_t TOTAL_MATH_RULES = NUM_CHANNELS * MAX_MATH_RULES_PER_CHANNEL;

static void ResetRuleLatchState(uint8_t channelIndex, uint8_t ruleIndex)
{
    hysteresisState[channelIndex][ruleIndex] = false;
    toggleState[channelIndex][ruleIndex] = false;
    togglePrevInput[channelIndex][ruleIndex] = false;
}

static bool IsUnaryOperator(MathOperator operation)
{
    return operation == MATH_NOT || operation == MATH_ABS || operation == MATH_TOGGLE;
}

static bool IsHysteresisOperator(MathOperator operation)
{
    return operation >= MATH_GREATER_THAN && operation <= MATH_LESS_EQUAL;
}

static bool IsValidOperand(uint8_t channelIndex, const MathOperand &operand)
{
    switch (operand.Type)
    {
    case MATH_OPERAND_CONSTANT:
        return isfinite(operand.Constant);
    case MATH_OPERAND_CHANNEL:
        return operand.Index < NUM_CHANNELS;
    case MATH_OPERAND_ANALOGUE_INPUT:
        return operand.Index < NUM_ANA_CHANNELS;
    case MATH_OPERAND_DIGITAL_INPUT:
        return operand.Index < NUM_DI_CHANNELS;
    case MATH_OPERAND_CAN_SIGNAL:
        return operand.Index < MAX_CUSTOM_CAN_SIGNALS;
    case MATH_OPERAND_SYSTEM_VALUE:
        return operand.Index < MATH_SYSTEM_VALUE_COUNT;
    case MATH_OPERAND_RULE:
        return channelIndex < NUM_CHANNELS && operand.Index < MathChannels[channelIndex].RuleCount;
    default:
        return false;
    }
}

MathRuntimeError ValidateMathChannelConfig(uint8_t channelIndex)
{
    if (channelIndex >= NUM_CHANNELS)
    {
        return MATH_ERROR_CONFIGURATION;
    }

    const MathChannelConfig &config = MathChannels[channelIndex];
    if (config.RuleCount == 0 || config.RuleCount > MAX_MATH_RULES_PER_CHANNEL ||
        config.RootRule >= config.RuleCount || config.OutputMode > MATH_OUTPUT_PWM)
    {
        return MATH_ERROR_CONFIGURATION;
    }
    return MATH_ERROR_NONE;
}

static bool ReadOperand(uint8_t channelIndex, const MathOperand &operand, float *value)
{
    if (value == nullptr)
    {
        return false;
    }

    switch (operand.Type)
    {
    case MATH_OPERAND_CONSTANT:
        *value = operand.Constant;
        return isfinite(*value);
    case MATH_OPERAND_CAN_SIGNAL:
        return ReadCustomCANSignal(operand.Index, millis(), value);
    case MATH_OPERAND_DIGITAL_INPUT:
        if (operand.Index >= NUM_DI_CHANNELS)
        {
            return false;
        }
        *value = ReadDigitalInputState(operand.Index) ? 1.0f : 0.0f;
        return true;
    case MATH_OPERAND_ANALOGUE_INPUT:
        if (operand.Index >= NUM_ANA_CHANNELS)
        {
            return false;
        }
        *value = ReadAnalogueInputValue(operand.Index);
        return isfinite(*value);
    case MATH_OPERAND_SYSTEM_VALUE:
        switch (operand.Index)
        {
        case MATH_SYSTEM_TEMPERATURE: *value = (float)SystemRuntimeParams.SystemTemperature; break;
        case MATH_SYSTEM_SIM_TEMPERATURE: *value = SystemRuntimeParams.SIMModuleTemp; break;
        case MATH_SYSTEM_IMU_TEMPERATURE: *value = SystemRuntimeParams.IMUTemp; break;
        case MATH_SYSTEM_BATTERY_VOLTAGE: *value = SystemRuntimeParams.VBatt; break;
        case MATH_SYSTEM_CURRENT: *value = SystemRuntimeParams.SystemCurrent; break;
        case MATH_SYSTEM_ACCEL_X: *value = accelX; break;
        case MATH_SYSTEM_ACCEL_Y: *value = accelY; break;
        case MATH_SYSTEM_ACCEL_Z: *value = accelZ; break;
        case MATH_SYSTEM_GYRO_X: *value = gyroX; break;
        case MATH_SYSTEM_GYRO_Y: *value = gyroY; break;
        case MATH_SYSTEM_GYRO_Z: *value = gyroZ; break;
        case MATH_SYSTEM_MAG_X: *value = magX; break;
        case MATH_SYSTEM_MAG_Y: *value = magY; break;
        case MATH_SYSTEM_MAG_Z: *value = magZ; break;
        default: return false;
        }
        return isfinite(*value);
    case MATH_OPERAND_RULE:
        if (operand.Index >= MathChannels[channelIndex].RuleCount ||
            MathChannelStates[channelIndex].Rules[operand.Index].Error != MATH_ERROR_NONE ||
            !MathChannelStates[channelIndex].Rules[operand.Index].Evaluated)
        {
            return false;
        }
        *value = MathChannelStates[channelIndex].Rules[operand.Index].Value;
        return true;
    case MATH_OPERAND_CHANNEL:
    {
        if (operand.Index >= NUM_CHANNELS || Channels[operand.Index].Enabled == 0)
        {
            return false;
        }
        if (Channels[operand.Index].ChanType == MATH)
        {
            uint8_t rootRule = MathChannels[operand.Index].RootRule;
            if (ValidateMathChannelConfig(operand.Index) != MATH_ERROR_NONE ||
                MathChannelStates[operand.Index].Rules[rootRule].Error != MATH_ERROR_NONE ||
                !MathChannelStates[operand.Index].Rules[rootRule].Evaluated)
            {
                return false;
            }
            *value = MathChannelStates[operand.Index].Rules[rootRule].Value;
            return true;
        }

        bool enabled = IsChannelRuntimeEnabled(operand.Index);
        switch (Channels[operand.Index].ChanType)
        {
        case DIG_PWM:
        case ANA_PWM:
        case CAN_PWM:
            *value = enabled ? Channels[operand.Index].PWMSetDuty : 0.0f;
            break;
        default:
            *value = enabled ? 1.0f : 0.0f;
            break;
        }
        return isfinite(*value);
    }
    default:
        return false;
    }
}

static bool ApplyOperation(uint8_t channelIndex, uint8_t ruleIndex, float left, float right, float *result)
{
    MathRuleConfig &config = MathChannels[channelIndex].Rules[ruleIndex];
    // Numeric zero is false; every finite non-zero value is true. Boolean outputs are 0 or 1.
    bool truthA = left != 0.0f;
    bool truthB = right != 0.0f;
    bool comparison = false;

    switch (config.Operation)
    {
    case MATH_ADD: *result = left + right; break;
    case MATH_SUBTRACT: *result = left - right; break;
    case MATH_MULTIPLY: *result = left * right; break;
    case MATH_DIVIDE:
        if (right == 0.0f)
        {
            return false;
        }
        *result = left / right;
        break;
    case MATH_GREATER_THAN: comparison = left > right; goto compare;
    case MATH_GREATER_EQUAL: comparison = left >= right; goto compare;
    case MATH_LESS_THAN: comparison = left < right; goto compare;
    case MATH_LESS_EQUAL: comparison = left <= right; goto compare;
    case MATH_EQUAL: comparison = left == right; goto compare;
    case MATH_NOT_EQUAL: comparison = left != right; goto compare;
    case MATH_AND: *result = (truthA && truthB) ? 1.0f : 0.0f; break;
    case MATH_OR: *result = (truthA || truthB) ? 1.0f : 0.0f; break;
    case MATH_XOR: *result = (truthA != truthB) ? 1.0f : 0.0f; break;
    case MATH_NOT: *result = truthA ? 0.0f : 1.0f; break;
    case MATH_MIN: *result = (left < right) ? left : right; break;
    case MATH_MAX: *result = (left > right) ? left : right; break;
    case MATH_ABS: *result = fabsf(left); break;
    case MATH_TOGGLE:
        // Edge-triggered latch: each rising edge of operand A flips the stored state.
        if (truthA && !togglePrevInput[channelIndex][ruleIndex])
        {
            toggleState[channelIndex][ruleIndex] = !toggleState[channelIndex][ruleIndex];
        }
        togglePrevInput[channelIndex][ruleIndex] = truthA;
        *result = toggleState[channelIndex][ruleIndex] ? 1.0f : 0.0f;
        break;
    default: return false;
    }
    return isfinite(*result);

compare:
    if (config.HysteresisEnabled && IsHysteresisOperator(config.Operation))
    {
        bool negativeGoing = config.OnThreshold < config.OffThreshold;
        bool turnOn = negativeGoing ? left <= config.OnThreshold : left >= config.OnThreshold;
        bool turnOff = negativeGoing ? left > config.OffThreshold : left < config.OffThreshold;
        if (turnOn)
        {
            hysteresisState[channelIndex][ruleIndex] = true;
        }
        else if (turnOff)
        {
            hysteresisState[channelIndex][ruleIndex] = false;
        }
        comparison = hysteresisState[channelIndex][ruleIndex];
    }
    *result = comparison ? 1.0f : 0.0f;
    return true;
}

void InitialiseMathChannelData()
{
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        MathChannels[i] = {};
        MathChannels[i].RuleCount = 1;
        MathChannels[i].RootRule = 0;
        MathChannels[i].OutputMode = MATH_OUTPUT_DIGITAL;
        for (uint8_t rule = 0; rule < MAX_MATH_RULES_PER_CHANNEL; rule++)
        {
            MathChannels[i].Rules[rule].Operation = MATH_ADD;
            MathChannels[i].Rules[rule].OperandA.Type = MATH_OPERAND_CONSTANT;
            MathChannels[i].Rules[rule].OperandB.Type = MATH_OPERAND_CONSTANT;
        }
        MathChannelStates[i] = {};
        for (uint8_t rule = 0; rule < MAX_MATH_RULES_PER_CHANNEL; rule++)
        {
            ResetRuleLatchState(i, rule);
        }
    }
}

void EvaluateMathChannels()
{
    bool pending[TOTAL_MATH_RULES] = {false};
    uint16_t remaining = 0;
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        MathChannelStates[i] = {};
        if (Channels[i].ChanType != MATH)
        {
            for (uint8_t rule = 0; rule < MAX_MATH_RULES_PER_CHANNEL; rule++)
            {
                ResetRuleLatchState(i, rule);
            }
            continue;
        }
        if (ValidateMathChannelConfig(i) != MATH_ERROR_NONE)
        {
            MathChannelStates[i].Error = MATH_ERROR_CONFIGURATION;
            for (uint8_t rule = 0; rule < MAX_MATH_RULES_PER_CHANNEL; rule++)
            {
                ResetRuleLatchState(i, rule);
            }
            continue;
        }
        for (uint8_t rule = 0; rule < MathChannels[i].RuleCount; rule++)
        {
            pending[i * MAX_MATH_RULES_PER_CHANNEL + rule] = true;
            remaining++;
        }
    }

    // Evaluate the flat channel/rule graph in fixed order. A stalled pass marks
    // cycles and every node blocked behind them without using recursion.
    while (remaining > 0)
    {
        bool progress = false;
        for (uint16_t node = 0; node < TOTAL_MATH_RULES; node++)
        {
            if (!pending[node])
            {
                continue;
            }
            uint8_t channel = node / MAX_MATH_RULES_PER_CHANNEL;
            uint8_t ruleIndex = node % MAX_MATH_RULES_PER_CHANNEL;
            MathRuleConfig &config = MathChannels[channel].Rules[ruleIndex];
            bool unary = IsUnaryOperator(config.Operation);
            bool hysteresis = config.HysteresisEnabled != 0;
            bool useOperandB = !unary && !hysteresis;
            bool validConfig = (uint8_t)config.Operation < MATH_OPERATOR_COUNT &&
                               config.HysteresisEnabled <= 1 &&
                               (!hysteresis || IsHysteresisOperator(config.Operation)) &&
                               isfinite(config.OnThreshold) && isfinite(config.OffThreshold) &&
                               IsValidOperand(channel, config.OperandA) &&
                               (!useOperandB || IsValidOperand(channel, config.OperandB));
            if (!validConfig)
            {
                MathChannelStates[channel].Rules[ruleIndex].Error = MATH_ERROR_CONFIGURATION;
                ResetRuleLatchState(channel, ruleIndex);
                pending[node] = false;
                remaining--;
                progress = true;
                continue;
            }

            const MathOperand *operands[] = {&config.OperandA, &config.OperandB};
            bool waiting = false;
            bool failedDependency = false;
            for (uint8_t op = 0; op < (useOperandB ? 2 : 1); op++)
            {
                const MathOperand &operand = *operands[op];
                uint8_t dependencyNode = 0;
                if (operand.Type == MATH_OPERAND_RULE)
                {
                    dependencyNode = channel * MAX_MATH_RULES_PER_CHANNEL + operand.Index;
                    if (dependencyNode < TOTAL_MATH_RULES && pending[dependencyNode])
                    {
                        waiting = true;
                    }
                    else if (dependencyNode >= TOTAL_MATH_RULES ||
                             MathChannelStates[channel].Rules[operand.Index].Error != MATH_ERROR_NONE)
                    {
                        failedDependency = true;
                    }
                }
                else if (operand.Type == MATH_OPERAND_CHANNEL && Channels[operand.Index].ChanType == MATH)
                {
                    if (ValidateMathChannelConfig(operand.Index) != MATH_ERROR_NONE)
                    {
                        failedDependency = true;
                    }
                    else
                    {
                        dependencyNode = operand.Index * MAX_MATH_RULES_PER_CHANNEL + MathChannels[operand.Index].RootRule;
                        if (pending[dependencyNode])
                        {
                            waiting = true;
                        }
                        else if (MathChannelStates[operand.Index].Rules[MathChannels[operand.Index].RootRule].Error != MATH_ERROR_NONE)
                        {
                            failedDependency = true;
                        }
                    }
                }
            }
            if (waiting)
            {
                continue;
            }
            if (failedDependency)
            {
                MathChannelStates[channel].Rules[ruleIndex].Error = MATH_ERROR_DEPENDENCY;
                ResetRuleLatchState(channel, ruleIndex);
            }
            else
            {
                float left = 0.0f;
                float right = 0.0f;
                float result = 0.0f;
                bool operandsValid = ReadOperand(channel, config.OperandA, &left);
                if (operandsValid && useOperandB)
                {
                    operandsValid = ReadOperand(channel, config.OperandB, &right);
                }
                if (!operandsValid)
                {
                    MathChannelStates[channel].Rules[ruleIndex].Error = MATH_ERROR_REFERENCE;
                    ResetRuleLatchState(channel, ruleIndex);
                }
                else if (!ApplyOperation(channel, ruleIndex, left, right, &result))
                {
                    MathChannelStates[channel].Rules[ruleIndex].Error = MATH_ERROR_EVALUATION;
                    ResetRuleLatchState(channel, ruleIndex);
                }
                else
                {
                    MathChannelStates[channel].Rules[ruleIndex].Value = result;
                }
            }
            MathChannelStates[channel].Rules[ruleIndex].Evaluated = 1;
            pending[node] = false;
            remaining--;
            progress = true;
        }
        if (!progress)
        {
            for (uint16_t node = 0; node < TOTAL_MATH_RULES; node++)
            {
                if (pending[node])
                {
                    uint8_t channel = node / MAX_MATH_RULES_PER_CHANNEL;
                    uint8_t ruleIndex = node % MAX_MATH_RULES_PER_CHANNEL;
                    MathChannelStates[channel].Rules[ruleIndex].Error = MATH_ERROR_DEPENDENCY;
                    MathChannelStates[channel].Rules[ruleIndex].Evaluated = 0;
                    MathChannelStates[channel].Rules[ruleIndex].Value = 0.0f;
                    ResetRuleLatchState(channel, ruleIndex);
                    pending[node] = false;
                    remaining--;
                }
            }
        }
    }

    for (uint8_t channel = 0; channel < NUM_CHANNELS; channel++)
    {
        if (Channels[channel].ChanType != MATH || MathChannelStates[channel].Error != MATH_ERROR_NONE)
        {
            continue;
        }
        uint8_t root = MathChannels[channel].RootRule;
        MathRuleRuntime &rootState = MathChannelStates[channel].Rules[root];
        MathChannelStates[channel].Value = rootState.Value;
        MathChannelStates[channel].Error = rootState.Error;
        MathChannelStates[channel].Evaluated = rootState.Evaluated;
    }
}

float GetMathChannelValue(uint8_t channelIndex)
{
    return channelIndex < NUM_CHANNELS ? MathChannelStates[channelIndex].Value : 0.0f;
}

uint8_t GetMathChannelError(uint8_t channelIndex)
{
    return channelIndex < NUM_CHANNELS ? MathChannelStates[channelIndex].Error : MATH_ERROR_REFERENCE;
}

bool ReadChannelOutputValue(uint8_t channelIndex, float *value)
{
    MathOperand operand = {};
    operand.Type = MATH_OPERAND_CHANNEL;
    operand.Index = channelIndex;
    return channelIndex < NUM_CHANNELS && ReadOperand(0, operand, value);
}