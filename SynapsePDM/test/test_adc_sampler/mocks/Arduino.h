#pragma once
#include <stdint.h>

typedef int32_t PinName;
constexpr PinName NC = -1;
struct ADC_TypeDef {};
extern ADC_TypeDef adc1;
#define ADC1 (&adc1)
extern const int PinMap_ADC[];
enum
{
    HAL_OK, HAL_ERROR, ADC_CLOCK_SYNC_PCLK_DIV4, ADC_RESOLUTION_12B,
    ADC_DATAALIGN_RIGHT, ADC_EOC_SINGLE_CONV, ADC_SOFTWARE_START,
    ADC_EXTERNALTRIGCONVEDGE_NONE, HAL_ADC_STATE_READY, ADC_SAMPLETIME_15CYCLES
};
constexpr uint32_t DISABLE = 0, HAL_UNLOCKED = 0;
struct ADC_InitTypeDef
{
    uint32_t ClockPrescaler, Resolution, DataAlign, ScanConvMode, EOCSelection;
    uint32_t ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode;
    uint32_t ExternalTrigConv, ExternalTrigConvEdge, DMAContinuousRequests;
};
struct ADC_HandleTypeDef
{
    ADC_TypeDef *Instance;
    ADC_InitTypeDef Init;
    uint32_t State, Lock;
};
struct ADC_ChannelConfTypeDef { uint32_t Channel, Rank, SamplingTime; };
uint32_t micros();
PinName digitalPinToPinName(uint32_t pin);
ADC_TypeDef *pinmap_peripheral(PinName pin, const int *);
void pinmap_pinout(PinName pin, const int *);
void EnableADCClock();
#define __HAL_RCC_ADC1_CLK_ENABLE() EnableADCClock()
uint32_t get_adc_channel(PinName pin, uint32_t *bank);
int HAL_ADC_Init(ADC_HandleTypeDef *);
int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *, ADC_ChannelConfTypeDef *);
int HAL_ADC_Start(ADC_HandleTypeDef *);
int HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
int HAL_ADC_Stop(ADC_HandleTypeDef *);
int HAL_ADC_DeInit(ADC_HandleTypeDef *);
