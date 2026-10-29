#include <unity.h>
#include "Arduino.h"
#include "../../src/AdcSampler.h"

ADC_TypeDef adc1, otherAdc;
const int PinMap_ADC[] = {0};
static int initCalls, configCalls, startCalls, pollCalls, stopCalls, deinitCalls;
static int failInit, failConfig, failStart, failPoll, failStop, failDeinit;
static uint32_t value, clockMicros;
static bool fixedValue;
uint32_t micros() { return clockMicros++; }
PinName digitalPinToPinName(uint32_t pin) { return pin < 15 ? static_cast<PinName>(pin) : NC; }
ADC_TypeDef *pinmap_peripheral(PinName pin, const int *) { return pin == 14 ? &otherAdc : ADC1; }
void pinmap_pinout(PinName, const int *) {}
void EnableADCClock() {}
uint32_t get_adc_channel(PinName pin, uint32_t *) { return pin; }
int HAL_ADC_Init(ADC_HandleTypeDef *adc)
{
    ++initCalls;
    TEST_ASSERT_EQUAL_PTR(ADC1, adc->Instance);
    TEST_ASSERT_EQUAL(ADC_CLOCK_SYNC_PCLK_DIV4, adc->Init.ClockPrescaler);
    TEST_ASSERT_EQUAL(DISABLE, adc->Init.DMAContinuousRequests);
    return failInit ? HAL_ERROR : HAL_OK;
}
int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *, ADC_ChannelConfTypeDef *config)
{
    ++configCalls;
    TEST_ASSERT_EQUAL(ADC_SAMPLETIME_15CYCLES, config->SamplingTime);
    return failConfig ? HAL_ERROR : HAL_OK;
}
int HAL_ADC_Start(ADC_HandleTypeDef *) { return ++startCalls == failStart ? HAL_ERROR : HAL_OK; }
int HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t)
{
    return ++pollCalls == failPoll ? HAL_ERROR : HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *) { return fixedValue ? value : value++; }
int HAL_ADC_Stop(ADC_HandleTypeDef *) { ++stopCalls; return failStop ? HAL_ERROR : HAL_OK; }
int HAL_ADC_DeInit(ADC_HandleTypeDef *) { ++deinitCalls; return failDeinit ? HAL_ERROR : HAL_OK; }
void setUp()
{
    initCalls = configCalls = startCalls = pollCalls = stopCalls = deinitCalls = 0;
    failInit = failConfig = failStart = failPoll = failStop = failDeinit = 0;
    value = 0;
    fixedValue = false;
}
void tearDown() {}
static void HundredDistinctSamplesUseOneInitialisation()
{
    uint16_t mean = 999;
    TEST_ASSERT_TRUE(ReadCurrentSenseMean(0, 100, &mean));
    TEST_ASSERT_EQUAL(49, mean);
    TEST_ASSERT_EQUAL(1, initCalls);
    TEST_ASSERT_EQUAL(100, startCalls);
    TEST_ASSERT_EQUAL(100, pollCalls);
    TEST_ASSERT_EQUAL(1, stopCalls);
    TEST_ASSERT_EQUAL(1, deinitCalls);
}
static void AllPinsSupportSingleSample()
{
    for (uint32_t pin = 0; pin < 14; ++pin)
    {
        fixedValue = true;
        value = pin + 10;
        uint16_t mean = 0;
        TEST_ASSERT_TRUE(ReadCurrentSenseMean(pin, 1, &mean));
        TEST_ASSERT_EQUAL(value, mean);
    }
}
static void FullScaleAccumulatorDoesNotOverflow()
{
    fixedValue = true;
    value = 4095;
    uint16_t mean = 0;
    TEST_ASSERT_TRUE(ReadCurrentSenseMean(0, UINT16_MAX, &mean));
    TEST_ASSERT_EQUAL(4095, mean);
}
static void PartialConversionFailureDoesNotPublishMean()
{
    failPoll = 5;
    uint16_t mean = 999;
    auto before = GetAdcSamplerDiagnostics();
    TEST_ASSERT_FALSE(ReadCurrentSenseMean(0, 100, &mean));
    TEST_ASSERT_EQUAL(999, mean);
    TEST_ASSERT_EQUAL(1, deinitCalls);
    auto after = GetAdcSamplerDiagnostics();
    TEST_ASSERT_EQUAL(before.Samples + 4, after.Samples);
    TEST_ASSERT_EQUAL(before.Failures + 1, after.Failures);
}
static void SetupAndStartFailuresReleaseHardware()
{
    for (int failure = 0; failure < 3; ++failure)
    {
        setUp();
        failInit = failure == 0;
        failConfig = failure == 1;
        failStart = failure == 2 ? 1 : 0;
        uint16_t mean = 999;
        TEST_ASSERT_FALSE(ReadCurrentSenseMean(0, 100, &mean));
        TEST_ASSERT_EQUAL(999, mean);
        TEST_ASSERT_EQUAL(1, stopCalls);
        TEST_ASSERT_EQUAL(1, deinitCalls);
    }
}
static void CleanupFailureIsNotSuccess()
{
    for (int failure = 0; failure < 2; ++failure)
    {
        setUp();
        failStop = failure == 0;
        failDeinit = failure == 1;
        uint16_t mean = 999;
        TEST_ASSERT_FALSE(ReadCurrentSenseMean(0, 1, &mean));
        TEST_ASSERT_EQUAL(999, mean);
    }
}
static void InvalidRequestsDoNotTouchHardware()
{
    uint16_t mean = 999;
    TEST_ASSERT_FALSE(ReadCurrentSenseMean(0, 0, &mean));
    TEST_ASSERT_FALSE(ReadCurrentSenseMean(0, 1, nullptr));
    TEST_ASSERT_FALSE(ReadCurrentSenseMean(14, 1, &mean));
    TEST_ASSERT_FALSE(ReadCurrentSenseMean(999, 1, &mean));
    TEST_ASSERT_EQUAL(999, mean);
    TEST_ASSERT_EQUAL(0, initCalls);
}
static void IOTimingHandlesDownloadAndWraparound()
{
    RecordIOServiceTiming(UINT32_MAX - 99, 10, false);
    RecordIOServiceTiming(100, 20, true);
    RecordIOServiceTiming(350, 15, true);
    auto result = GetAdcSamplerDiagnostics();
    TEST_ASSERT_EQUAL(3, result.IOServices);
    TEST_ASSERT_EQUAL(20, result.MaxIOMicros);
    TEST_ASSERT_EQUAL(250, result.MaxIOGapMicros);
    TEST_ASSERT_EQUAL(2, result.DownloadIOServices);
    TEST_ASSERT_EQUAL(250, result.MaxDownloadIOGapMicros);
}
int main()
{
    UNITY_BEGIN();
    RUN_TEST(HundredDistinctSamplesUseOneInitialisation);
    RUN_TEST(AllPinsSupportSingleSample);
    RUN_TEST(FullScaleAccumulatorDoesNotOverflow);
    RUN_TEST(PartialConversionFailureDoesNotPublishMean);
    RUN_TEST(SetupAndStartFailuresReleaseHardware);
    RUN_TEST(CleanupFailureIsNotSuccess);
    RUN_TEST(InvalidRequestsDoNotTouchHardware);
    RUN_TEST(IOTimingHandlesDownloadAndWraparound);
    return UNITY_END();
}
