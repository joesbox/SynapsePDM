#include <Arduino.h>
#include <unity.h>
#include <float.h>
#include <math.h>
#include "../../src/MathHandler.cpp"
#include "../../src/CustomCAN.cpp"

ChannelConfig Channels[NUM_CHANNELS] = {};
MathChannelConfig MathChannels[NUM_CHANNELS] = {};
MathChannelRuntime MathChannelStates[NUM_CHANNELS] = {};
SystemRuntime SystemRuntimeParams = {};

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;

static bool runtimeEnabled[NUM_CHANNELS] = {false};
static float analogueValues[NUM_ANA_CHANNELS] = {0.0f};
static bool digitalValues[NUM_DI_CHANNELS] = {false};

bool ReadDigitalInputState(uint8_t inputIndex)
{
    return inputIndex < NUM_DI_CHANNELS && digitalValues[inputIndex];
}

bool IsChannelRuntimeEnabled(uint8_t channelIndex)
{
    return channelIndex < NUM_CHANNELS && runtimeEnabled[channelIndex];
}

float ReadAnalogueInputValue(int inputIndex)
{
    return inputIndex >= 0 && inputIndex < NUM_ANA_CHANNELS ? analogueValues[inputIndex] : NAN;
}

static MathOperand Constant(float value)
{
    MathOperand operand = {};
    operand.Type = MATH_OPERAND_CONSTANT;
    operand.Constant = value;
    return operand;
}

static MathOperand Reference(MathOperandType type, uint8_t index)
{
    MathOperand operand = {};
    operand.Type = type;
    operand.Index = index;
    return operand;
}

static void SetExpression(uint8_t channel, MathOperator operation, MathOperand left, MathOperand right)
{
    Channels[channel].ChanType = MATH;
    Channels[channel].Enabled = 1;
    MathChannels[channel].RuleCount = 1;
    MathChannels[channel].RootRule = 0;
    MathRuleConfig &rule = MathChannels[channel].Rules[0];
    rule.Operation = operation;
    rule.OperandA = left;
    rule.OperandB = right;
}

void setUp()
{
    memset(Channels, 0, sizeof(Channels));
    memset(MathChannels, 0, sizeof(MathChannels));
    memset(MathChannelStates, 0, sizeof(MathChannelStates));
    memset(runtimeEnabled, 0, sizeof(runtimeEnabled));
    memset(analogueValues, 0, sizeof(analogueValues));
    memset(digitalValues, 0, sizeof(digitalValues));
    memset(&SystemRuntimeParams, 0, sizeof(SystemRuntimeParams));
    accelX = accelY = accelZ = 0.0f;
    gyroX = gyroY = gyroZ = 0.0f;
    magX = magY = magZ = 0.0f;
    InitialiseMathChannelData();
    InitialiseCustomCANData();
}

void tearDown() {}

void test_arithmetic_and_utility_operators()
{
    SetExpression(0, MATH_ADD, Constant(7.0f), Constant(3.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(10.0f, MathChannelStates[0].Value);

    MathChannels[0].Rules[0].Operation = MATH_SUBTRACT;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(4.0f, MathChannelStates[0].Value);

    MathChannels[0].Rules[0].Operation = MATH_MULTIPLY;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(21.0f, MathChannelStates[0].Value);

    MathChannels[0].Rules[0].Operation = MATH_DIVIDE;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(7.0f / 3.0f, MathChannelStates[0].Value);

    MathChannels[0].Rules[0].Operation = MATH_MIN;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(3.0f, MathChannelStates[0].Value);

    MathChannels[0].Rules[0].Operation = MATH_MAX;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(7.0f, MathChannelStates[0].Value);

    MathChannels[0].Rules[0].Operation = MATH_ABS;
    MathChannels[0].Rules[0].OperandA = Constant(-7.0f);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(7.0f, MathChannelStates[0].Value);
}

void test_comparison_operators()
{
    SetExpression(0, MATH_GREATER_THAN, Constant(3.0f), Constant(2.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    MathChannels[0].Rules[0].Operation = MATH_GREATER_EQUAL;
    MathChannels[0].Rules[0].OperandA = Constant(2.0f);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    MathChannels[0].Rules[0].Operation = MATH_LESS_THAN;
    MathChannels[0].Rules[0].OperandA = Constant(1.0f);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    MathChannels[0].Rules[0].Operation = MATH_LESS_EQUAL;
    MathChannels[0].Rules[0].OperandA = Constant(2.0f);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    MathChannels[0].Rules[0].Operation = MATH_EQUAL;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    MathChannels[0].Rules[0].Operation = MATH_NOT_EQUAL;
    MathChannels[0].Rules[0].OperandA = Constant(3.0f);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
}

void test_boolean_operators_and_nonzero_conversion()
{
    SetExpression(0, MATH_AND, Constant(-0.5f), Constant(2.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    MathChannels[0].Rules[0].Operation = MATH_OR;
    MathChannels[0].Rules[0].OperandA = Constant(0.0f);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    MathChannels[0].Rules[0].Operation = MATH_XOR;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    MathChannels[0].Rules[0].Operation = MATH_NOT;
    MathChannels[0].Rules[0].OperandA = Constant(0.0f);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
}

void test_channel_analogue_system_and_chained_operands()
{
    Channels[1].Enabled = 1;
    Channels[1].ChanType = DIG;
    runtimeEnabled[1] = true;
    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_CHANNEL, 1), Constant(2.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(3.0f, MathChannelStates[0].Value);

    Channels[1].ChanType = DIG_PWM;
    Channels[1].PWMSetDuty = 37;
    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_CHANNEL, 1), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(37.0f, MathChannelStates[0].Value);

    analogueValues[2] = 86.5f;
    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_ANALOGUE_INPUT, 2), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(86.5f, MathChannelStates[0].Value);

    SystemRuntimeParams.VBatt = 13.8f;
    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_SYSTEM_VALUE, MATH_SYSTEM_BATTERY_VOLTAGE), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(13.8f, MathChannelStates[0].Value);

    SetExpression(0, MATH_MULTIPLY, Reference(MATH_OPERAND_CHANNEL, 1), Constant(2.0f));
    SetExpression(1, MATH_ADD, Constant(20.0f), Constant(22.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(84.0f, MathChannelStates[0].Value);
}

void test_digital_input_operands()
{
    TEST_ASSERT_EQUAL_UINT8(3, MATH_OPERAND_SYSTEM_VALUE);
    TEST_ASSERT_EQUAL_UINT8(4, MATH_OPERAND_RULE);
    TEST_ASSERT_EQUAL_UINT8(5, MATH_OPERAND_DIGITAL_INPUT);

    for (uint8_t input = 0; input < NUM_DI_CHANNELS; input++)
    {
        for (uint8_t state = 0; state < 2; state++)
        {
            digitalValues[input] = state != 0;
            SetExpression(0, MATH_EQUAL, Reference(MATH_OPERAND_DIGITAL_INPUT, input), Constant((float)state));
            EvaluateMathChannels();
            TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_NONE, MathChannelStates[0].Error);
            TEST_ASSERT_EQUAL_UINT8(1, MathChannelStates[0].Evaluated);
            TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);

            SetExpression(0, MATH_ADD, Constant(2.0f), Reference(MATH_OPERAND_DIGITAL_INPUT, input));
            EvaluateMathChannels();
            TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_NONE, MathChannelStates[0].Error);
            TEST_ASSERT_EQUAL_FLOAT(2.0f + state, MathChannelStates[0].Value);
        }
    }

    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_DIGITAL_INPUT, NUM_DI_CHANNELS), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_CONFIGURATION, MathChannelStates[0].Error);

    SetExpression(0, MATH_ADD, Constant(0.0f), Reference(MATH_OPERAND_DIGITAL_INPUT, NUM_DI_CHANNELS));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_CONFIGURATION, MathChannelStates[0].Error);
}

void test_imu_axis_system_operands()
{
    const MathSystemValue axes[] = {
        MATH_SYSTEM_ACCEL_X, MATH_SYSTEM_ACCEL_Y, MATH_SYSTEM_ACCEL_Z,
        MATH_SYSTEM_GYRO_X, MATH_SYSTEM_GYRO_Y, MATH_SYSTEM_GYRO_Z,
        MATH_SYSTEM_MAG_X, MATH_SYSTEM_MAG_Y, MATH_SYSTEM_MAG_Z};
    float *values[] = {&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ, &magX, &magY, &magZ};

    TEST_ASSERT_EQUAL_UINT8(4, MATH_SYSTEM_CURRENT);
    TEST_ASSERT_EQUAL_UINT8(14, MATH_SYSTEM_VALUE_COUNT);

    for (uint8_t axis = 0; axis < 9; axis++)
    {
        TEST_ASSERT_EQUAL_UINT8(5 + axis, axes[axis]);
        *values[axis] = -0.25f * (axis + 1);
    }

    for (uint8_t axis = 0; axis < 9; axis++)
    {
        float expected = -0.25f * (axis + 1);
        SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_SYSTEM_VALUE, axes[axis]), Constant(0.0f));
        EvaluateMathChannels();
        TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_NONE, MathChannelStates[0].Error);
        TEST_ASSERT_EQUAL_FLOAT(expected, MathChannelStates[0].Value);

        *values[axis] = -expected;
        SetExpression(0, MATH_ADD, Constant(0.0f), Reference(MATH_OPERAND_SYSTEM_VALUE, axes[axis]));
        EvaluateMathChannels();
        TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_NONE, MathChannelStates[0].Error);
        TEST_ASSERT_EQUAL_FLOAT(-expected, MathChannelStates[0].Value);

        *values[axis] = NAN;
        EvaluateMathChannels();
        TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_REFERENCE, MathChannelStates[0].Error);

        *values[axis] = INFINITY;
        EvaluateMathChannels();
        TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_REFERENCE, MathChannelStates[0].Error);
    }

    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_SYSTEM_VALUE, MATH_SYSTEM_VALUE_COUNT), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_CONFIGURATION, MathChannelStates[0].Error);
}

void test_three_rule_signal_composition()
{
    analogueValues[0] = 95.0f;
    analogueValues[1] = 100.0f;
    Channels[0].ChanType = MATH;
    Channels[0].Enabled = 1;
    MathChannels[0].RuleCount = 3;
    MathChannels[0].RootRule = 2;

    MathChannels[0].Rules[0].Operation = MATH_GREATER_THAN;
    MathChannels[0].Rules[0].OperandA = Reference(MATH_OPERAND_ANALOGUE_INPUT, 0);
    MathChannels[0].Rules[0].OperandB = Constant(90.0f);

    MathChannels[0].Rules[1].Operation = MATH_GREATER_THAN;
    MathChannels[0].Rules[1].OperandA = Reference(MATH_OPERAND_ANALOGUE_INPUT, 1);
    MathChannels[0].Rules[1].OperandB = Constant(110.0f);

    MathChannels[0].Rules[2].Operation = MATH_OR;
    MathChannels[0].Rules[2].OperandA = Reference(MATH_OPERAND_RULE, 0);
    MathChannels[0].Rules[2].OperandB = Reference(MATH_OPERAND_RULE, 1);

    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Rules[0].Value);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, MathChannelStates[0].Rules[1].Value);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
}

void test_eight_rule_capacity_and_local_cycle()
{
    Channels[0].ChanType = MATH;
    Channels[0].Enabled = 1;
    MathChannels[0].RuleCount = MAX_MATH_RULES_PER_CHANNEL;
    MathChannels[0].RootRule = MAX_MATH_RULES_PER_CHANNEL - 1;
    MathChannels[0].Rules[0].OperandA = Constant(4.0f);
    MathChannels[0].Rules[0].OperandB = Constant(0.0f);

    for (uint8_t rule = 1; rule < MAX_MATH_RULES_PER_CHANNEL; rule++)
    {
        MathChannels[0].Rules[rule].OperandA = Reference(MATH_OPERAND_RULE, rule - 1);
        MathChannels[0].Rules[rule].OperandB = Constant(1.0f);
    }

    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(11.0f, MathChannelStates[0].Value);

    MathChannels[0].RootRule = 0;
    MathChannels[0].Rules[0].OperandA = Reference(MATH_OPERAND_RULE, 1);
    MathChannels[0].Rules[1].OperandA = Reference(MATH_OPERAND_RULE, 0);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_DEPENDENCY, MathChannelStates[0].Error);
}

void test_per_rule_hysteresis_thresholds()
{
    Channels[0].OnThreshold = 10.0f;
    Channels[0].OffThreshold = 8.0f;
    SetExpression(0, MATH_GREATER_THAN, Constant(10.0f), Constant(100.0f));
    MathChannels[0].Rules[0].HysteresisEnabled = 1;
    MathChannels[0].Rules[0].OnThreshold = 10.0f;
    MathChannels[0].Rules[0].OffThreshold = 8.0f;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);

    MathChannels[0].Rules[0].OperandA = Constant(9.0f);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);

    MathChannels[0].Rules[0].OperandA = Constant(7.0f);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, MathChannelStates[0].Value);
}

void test_invalid_references_divide_by_zero_cycles_and_float_boundaries()
{
    SetExpression(0, MATH_DIVIDE, Constant(1.0f), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_EVALUATION, MathChannelStates[0].Error);

    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_CHANNEL, NUM_CHANNELS), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_CONFIGURATION, MathChannelStates[0].Error);

    Channels[1].Enabled = 0;
    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_CHANNEL, 1), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_REFERENCE, MathChannelStates[0].Error);

    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_CHANNEL, 1), Constant(0.0f));
    SetExpression(1, MATH_ADD, Reference(MATH_OPERAND_CHANNEL, 0), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_DEPENDENCY, MathChannelStates[0].Error);
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_DEPENDENCY, MathChannelStates[1].Error);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, MathChannelStates[0].Value);

    SetExpression(0, MATH_ADD, Constant(FLT_MAX), Constant(FLT_MAX));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_EVALUATION, MathChannelStates[0].Error);

    SetExpression(0, MATH_ADD, Constant(NAN), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_CONFIGURATION, MathChannelStates[0].Error);
}

void test_global_can_signal_shared_by_math_channels_and_timeout()
{
    CustomCANMessageConfig message = {0x180, 1000, 8, CUSTOM_CAN_MESSAGE_ENABLED};
    CustomCANSignalConfig signal = {0, 5, 12, CUSTOM_CAN_SIGNAL_ENABLED, 0.5f, 10.0f};
    TEST_ASSERT_TRUE(SetCustomCANMessage(0, message));
    TEST_ASSERT_TRUE(SetCustomCANSignal(99, signal));
    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_CAN_SIGNAL, 99), Constant(1));
    SetExpression(1, MATH_MULTIPLY, Reference(MATH_OPERAND_CAN_SIGNAL, 99), Constant(2));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_REFERENCE, MathChannelStates[0].Error);
    uint8_t data[8] = {0x80, 0x0C};
    ProcessCustomCANFrame(0x180, false, false, data, 8, millis());
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(61.0f, MathChannelStates[0].Value);
    TEST_ASSERT_EQUAL_FLOAT(120.0f, MathChannelStates[1].Value);
    ProcessCustomCANFrame(0x180, false, false, data, 8, millis() - 1000);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_REFERENCE, MathChannelStates[0].Error);
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_REFERENCE, MathChannelStates[1].Error);
    ProcessCustomCANFrame(0x180, false, false, data, 8, millis());
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_NONE, MathChannelStates[0].Error);
    SetExpression(0, MATH_ADD, Reference(MATH_OPERAND_CAN_SIGNAL, 100), Constant(0));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_CONFIGURATION, MathChannelStates[0].Error);
}

void test_toggle_operator_momentary_button()
{
    TEST_ASSERT_EQUAL_UINT8(16, MATH_ABS);
    TEST_ASSERT_EQUAL_UINT8(17, MATH_TOGGLE);
    TEST_ASSERT_EQUAL_UINT8(18, MATH_OPERATOR_COUNT);

    SetExpression(0, MATH_TOGGLE, Reference(MATH_OPERAND_DIGITAL_INPUT, 0), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_NONE, MathChannelStates[0].Error);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, MathChannelStates[0].Value);

    // Press: the rising edge toggles on; holding or releasing keeps the state.
    digitalValues[0] = true;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
    digitalValues[0] = false;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);

    // Second press toggles off again.
    digitalValues[0] = true;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, MathChannelStates[0].Value);

    // TOGGLE is unary and not a hysteresis operator.
    MathChannels[0].Rules[0].HysteresisEnabled = 1;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_CONFIGURATION, MathChannelStates[0].Error);

    // Errors clear the latch; recovery re-arms edge detection.
    MathChannels[0].Rules[0].HysteresisEnabled = 0;
    EvaluateMathChannels();
    TEST_ASSERT_EQUAL_UINT8(MATH_ERROR_NONE, MathChannelStates[0].Error);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, MathChannelStates[0].Value);
}

void test_read_channel_output_value_for_custom_can_tx()
{
    float value = NAN;
    TEST_ASSERT_FALSE(ReadChannelOutputValue(NUM_CHANNELS, &value));
    TEST_ASSERT_FALSE(ReadChannelOutputValue(1, &value));

    Channels[1].Enabled = 1;
    Channels[1].ChanType = DIG;
    runtimeEnabled[1] = true;
    TEST_ASSERT_TRUE(ReadChannelOutputValue(1, &value));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, value);
    runtimeEnabled[1] = false;
    TEST_ASSERT_TRUE(ReadChannelOutputValue(1, &value));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, value);

    Channels[1].ChanType = DIG_PWM;
    Channels[1].PWMSetDuty = 42;
    runtimeEnabled[1] = true;
    TEST_ASSERT_TRUE(ReadChannelOutputValue(1, &value));
    TEST_ASSERT_EQUAL_FLOAT(42.0f, value);

    // MATH channels expose their evaluated root value; errors read as unavailable.
    SetExpression(0, MATH_ADD, Constant(2.0f), Constant(3.0f));
    EvaluateMathChannels();
    TEST_ASSERT_TRUE(ReadChannelOutputValue(0, &value));
    TEST_ASSERT_EQUAL_FLOAT(5.0f, value);
    SetExpression(0, MATH_DIVIDE, Constant(1.0f), Constant(0.0f));
    EvaluateMathChannels();
    TEST_ASSERT_FALSE(ReadChannelOutputValue(0, &value));
}

void setup()
{
    delay(1000);
    UNITY_BEGIN();
    RUN_TEST(test_arithmetic_and_utility_operators);
    RUN_TEST(test_comparison_operators);
    RUN_TEST(test_boolean_operators_and_nonzero_conversion);
    RUN_TEST(test_channel_analogue_system_and_chained_operands);
    RUN_TEST(test_digital_input_operands);
    RUN_TEST(test_imu_axis_system_operands);
    RUN_TEST(test_three_rule_signal_composition);
    RUN_TEST(test_eight_rule_capacity_and_local_cycle);
    RUN_TEST(test_per_rule_hysteresis_thresholds);
    RUN_TEST(test_invalid_references_divide_by_zero_cycles_and_float_boundaries);
    RUN_TEST(test_global_can_signal_shared_by_math_channels_and_timeout);
    RUN_TEST(test_toggle_operator_momentary_button);
    RUN_TEST(test_read_channel_output_value_for_custom_can_tx);
    UNITY_END();
}

void loop() {}