#include "AdcSampler.h"
#include <Arduino.h>
#include <analog.h>

static AdcSamplerDiagnostics diagnostics = {};
static uint32_t lastIOStart = 0;
static bool haveIOStart = false;
static bool lastIOWasDownload = false;

bool ReadCurrentSenseMean(uint32_t pin, uint16_t sampleCount, uint16_t *mean)
{
    uint32_t startedAt = micros();
    diagnostics.Batches++;
    bool succeeded = false;
    uint32_t sum = 0;
    PinName adcPin = digitalPinToPinName(pin);
    ADC_HandleTypeDef adc = {};

    if (mean != nullptr && sampleCount > 0 && adcPin != NC &&
        pinmap_peripheral(adcPin, PinMap_ADC) == ADC1)
    {
        adc.Instance = ADC1;
        adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
        adc.Init.Resolution = ADC_RESOLUTION_12B;
        adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        adc.Init.ScanConvMode = DISABLE;
        adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
        adc.Init.ContinuousConvMode = DISABLE;
        adc.Init.NbrOfConversion = 1;
        adc.Init.DiscontinuousConvMode = DISABLE;
        adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
        adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
        adc.Init.DMAContinuousRequests = DISABLE;

        // Arduino's MSP callback uses its private last-read pin.
        pinmap_pinout(adcPin, PinMap_ADC);
        __HAL_RCC_ADC1_CLK_ENABLE();
        adc.State = HAL_ADC_STATE_READY;
        adc.Lock = HAL_UNLOCKED;

        uint32_t bank = 0;
        ADC_ChannelConfTypeDef channel = {};
        channel.Channel = get_adc_channel(adcPin, &bank);
        channel.Rank = 1;
        channel.SamplingTime = ADC_SAMPLETIME_15CYCLES;

        if (HAL_ADC_Init(&adc) == HAL_OK && HAL_ADC_ConfigChannel(&adc, &channel) == HAL_OK)
        {
            succeeded = true;
            for (uint16_t sample = 0; sample < sampleCount; sample++)
            {
                if (HAL_ADC_Start(&adc) != HAL_OK || HAL_ADC_PollForConversion(&adc, 1) != HAL_OK)
                {
                    succeeded = false;
                    break;
                }
                sum += HAL_ADC_GetValue(&adc);
                diagnostics.Samples++;
            }
        }
        bool stopped = HAL_ADC_Stop(&adc) == HAL_OK;
        bool deinitialised = HAL_ADC_DeInit(&adc) == HAL_OK;
        succeeded = succeeded && stopped && deinitialised;
    }

    uint32_t duration = micros() - startedAt;
    if (duration > diagnostics.MaxBatchMicros)
    {
        diagnostics.MaxBatchMicros = duration;
    }
    if (!succeeded)
    {
        diagnostics.Failures++;
        return false;
    }
    *mean = static_cast<uint16_t>(sum / sampleCount);
    return true;
}

void RecordIOServiceTiming(uint32_t startedAt, uint32_t duration, bool downloading)
{
    diagnostics.IOServices++;
    if (duration > diagnostics.MaxIOMicros)
    {
        diagnostics.MaxIOMicros = duration;
    }
    uint32_t gap = haveIOStart ? startedAt - lastIOStart : 0;
    if (gap > diagnostics.MaxIOGapMicros)
    {
        diagnostics.MaxIOGapMicros = gap;
    }
    if (downloading)
    {
        diagnostics.DownloadIOServices++;
        if (duration > diagnostics.MaxDownloadIOMicros)
        {
            diagnostics.MaxDownloadIOMicros = duration;
        }
        if (lastIOWasDownload && gap > diagnostics.MaxDownloadIOGapMicros)
        {
            diagnostics.MaxDownloadIOGapMicros = gap;
        }
    }
    lastIOStart = startedAt;
    haveIOStart = true;
    lastIOWasDownload = downloading;
}

AdcSamplerDiagnostics GetAdcSamplerDiagnostics()
{
    return diagnostics;
}
