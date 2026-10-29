#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H

#include <stdint.h>

struct AdcSamplerDiagnostics
{
    uint32_t Batches;
    uint32_t Samples;
    uint32_t Failures;
    uint32_t MaxBatchMicros;
    uint32_t IOServices;
    uint32_t MaxIOMicros;
    uint32_t MaxIOGapMicros;
    uint32_t DownloadIOServices;
    uint32_t MaxDownloadIOMicros;
    uint32_t MaxDownloadIOGapMicros;
};

bool ReadCurrentSenseMean(uint32_t pin, uint16_t sampleCount, uint16_t *mean);
void RecordIOServiceTiming(uint32_t startedAt, uint32_t duration, bool downloading);
AdcSamplerDiagnostics GetAdcSamplerDiagnostics();

#endif
