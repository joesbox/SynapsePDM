/*  OutputHandler.cpp Output handler deals with channel output control.
    Specifically applies to the Infineon BTS50010 High-Side Driver
    Copyright (c) 2023 Joe Mann.  All right reserved.

    This work is licensed under the Creative Commons
    Attribution-NonCommercial-ShareAlike 4.0 International License.
    To view a copy of this license, visit
    https://creativecommons.org/licenses/by-nc-sa/4.0/ or send a
    letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

    You are free to:
    - Share: Copy and redistribute the material in any medium or format.
    - Adapt: Remix, transform, and build upon the material.

    Under the following terms:
    - Attribution: You must give appropriate credit, provide a link to the license,
      and indicate if changes were made. You may do so in any reasonable manner,
      but not in any way that suggests the licensor endorses you or your use.
    - NonCommercial: You may not use the material for commercial purposes.
    - ShareAlike: If you remix, transform, or build upon the material,
      you must distribute your contributions under the same license as the original.

    DISCLAIMER: This software is provided "as is," without warranty of any kind,
    express or implied, including but not limited to the warranties of
    merchantability, fitness for a particular purpose, and noninfringement.
    In no event shall the authors or copyright holders be liable for any claim,
    damages, or other liability, whether in an action of contract, tort, or otherwise,
    arising from, out of, or in connection with the software or the use or
    other dealings in the software.
*/

#include "OutputHandler.h"

// Pins to update
const uint16_t GPIOG_PINS[] = {GPIO_PIN_10, GPIO_PIN_9, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_3, GPIO_PIN_2}; // Outputs 1 to 7
const uint8_t NUM_PINS_G = sizeof(GPIOG_PINS) / sizeof(GPIOG_PINS[0]);

const uint16_t GPIOF_PINS[] = {GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_12, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0}; // Outputs 8 to 14
const uint8_t NUM_PINS_F = sizeof(GPIOF_PINS) / sizeof(GPIOF_PINS[0]);

// DMA buffer for multiple pins
uint32_t pwmBufferG[100] = {0};
uint32_t pwmBufferF[100] = {0};

// Independent duty cycle tracking
uint8_t dutyCycles[14] = {0};

// Timer and DMA handles
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim1;

volatile uint8_t analogCounter;
uint analogValues[NUM_CHANNELS][ANALOG_READ_SAMPLES];

// Channel number used to identify associated channel
int channelNum;

/// @brief Handle output control
void InitialiseOutputs()
{
  setupGPIO();
  configureDMA();
  configureTimer();

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    updatePWMDutyCycle(i, 0);
  }

  // Reset the counters
  analogCounter = 0;
}

void setupGPIO()
{
  __HAL_RCC_GPIOG_CLK_ENABLE();

  GPIO_InitTypeDef GPIOG_InitStruct = {0};
  GPIOG_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2;
  GPIOG_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIOG_InitStruct.Pull = GPIO_NOPULL;
  GPIOG_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIOG_InitStruct);

  __HAL_RCC_GPIOF_CLK_ENABLE();

  GPIO_InitTypeDef GPIOF_InitStruct = {0};
  GPIOF_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13 | GPIO_PIN_12 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0;
  GPIOF_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIOF_InitStruct.Pull = GPIO_NOPULL;
  GPIOF_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIOF_InitStruct);
}

void configureDMA()
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  // First DMA (Stream 1, Channel 7)
  static DMA_HandleTypeDef hdma;
  hdma.Instance = DMA2_Stream1;
  hdma.Init.Channel = DMA_CHANNEL_7;
  hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma.Init.MemInc = DMA_MINC_ENABLE;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma.Init.Mode = DMA_CIRCULAR;
  hdma.Init.Priority = DMA_PRIORITY_HIGH;
  hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  HAL_DMA_Init(&hdma);
  htim8.hdma[TIM_DMA_ID_UPDATE] = &hdma;

  HAL_DMA_Start(&hdma, (uint32_t)pwmBufferG, (uint32_t)&GPIOG->BSRR, 100);

  // Second DMA (Stream 5, Channel 6)
  static DMA_HandleTypeDef hdma_tim1_up;
  hdma_tim1_up.Instance = DMA2_Stream5;
  hdma_tim1_up.Init.Channel = DMA_CHANNEL_6;
  hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim1_up.Init.Mode = DMA_CIRCULAR;
  hdma_tim1_up.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim1_up.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  HAL_DMA_Init(&hdma_tim1_up);
  htim1.hdma[TIM_DMA_ID_UPDATE] = &hdma;

  HAL_DMA_Start(&hdma_tim1_up, (uint32_t)pwmBufferF, (uint32_t)&GPIOF->BSRR, 100);
}

void configureTimer()
{
  __HAL_RCC_TIM8_CLK_ENABLE();

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 84 - 1; // 84MHz / 84 = 1MHz
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100 - 1; // 1MHz / 100 = 10kHz
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  HAL_TIM_Base_Init(&htim8);
  HAL_TIM_Base_Start(&htim8);

  __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_UPDATE);

  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84 - 1; // 84MHz / 84 = 1MHz
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100 - 1; // 1MHz / 100 = 10kHz
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  HAL_TIM_Base_Init(&htim1);
  HAL_TIM_Base_Start(&htim1);

  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
}

// Setup PWM buffer
void updatePWMDutyCycle(uint8_t pinIndex, uint8_t dutyCycle)
{
  if (pinIndex >= NUM_PINS_G + NUM_PINS_F)
    return; // Ensure valid index

  dutyCycles[pinIndex] = dutyCycle; // Store the new duty cycle

  for (int i = 0; i < 100; i++)
  {
    uint32_t setMask;
    uint32_t resetMask;

    if (pinIndex < NUM_PINS_G)
    {
      setMask = GPIOG_PINS[pinIndex];
      resetMask = GPIOG_PINS[pinIndex] << 16; // BSRR reset value is the pin shifted left by 16
      if (i < dutyCycle)
      {
        pwmBufferG[i] |= setMask; // Set pin high
        pwmBufferG[i] &= ~resetMask;
      }
      else
      {
        pwmBufferG[i] |= resetMask; // Set pin low
        pwmBufferG[i] &= ~setMask;
      }
    }
    else
    {
      setMask = GPIOF_PINS[pinIndex - NUM_PINS_G];
      resetMask = GPIOF_PINS[pinIndex - NUM_PINS_G] << 16; // BSRR reset value is the pin shifted left by 16
      if (i < dutyCycle)
      {
        pwmBufferF[i] |= setMask; // Set pin high
        pwmBufferF[i] &= ~resetMask;
      }
      else
      {
        pwmBufferF[i] |= resetMask; // Set pin low
        pwmBufferF[i] &= ~setMask;
      }
    }
  }
}

/// @brief Update PWM or digital outputs
void UpdateOutputs()
{
  // Check the type of channel we're dealing with (digital or PWM) and handle output accordingly
  for (int i = 0; i < NUM_CHANNELS; i++)
  {

    switch (Channels[i].ChanType)
    {
    case DIG_PWM:
      if (Channels[i].Enabled)
      {
        // Calculate the adjusted PWM for volage/average power
        float squared = (VBATT_NOMINAL / SystemParams.VBatt) * (VBATT_NOMINAL / SystemParams.VBatt);
        int pwmActual = round(Channels[i].PWMSetDuty * squared);

        // PWM range check
        if (pwmActual > 100)
        {
          pwmActual = 100;
        }
        else if (pwmActual < 0)
        {
          pwmActual = 0;
        }
        updatePWMDutyCycle(i, pwmActual);

        // TODO: Calibrate current readings for the BTS50025
        int sum = 0;
        uint8_t total = 0;
        for (int j = 0; j < ANALOG_READ_SAMPLES; j++)
        {
          sum += analogRead(Channels[i].CurrentSensePin);
          total++;
        }
        float analogMean = 0.0f;
        if (total)
        {
          analogMean = sum / total;
          Channels[i].AnalogRaw = analogMean;
        }

        float isVoltage = (Channels[i].AnalogRaw / ADCres) * V_REF;
        float amps = (PWM_M * Channels[i].AnalogRaw) + PWM_C;

        if (Channels[i].AnalogRaw < 5)
        {
          // No current detected, set to 0
          amps = 0.0;
        }

        // Check for fault condition and current thresholds
        if (isVoltage > FAULT_THRESHOLD)
        {
          Channels[i].ErrorFlags |= IS_FAULT;
        }
        else if (amps > Channels[i].CurrentThresholdHigh)
        {
          Channels[i].ErrorFlags |= CHN_OVERCURRENT_RANGE;
        }
        else if (amps < Channels[i].CurrentThresholdLow)
        {
          Channels[i].ErrorFlags |= CHN_UNDERCURRENT_RANGE;
        }
        else if (amps > Channels[i].CurrentLimitHigh)
        {
          Channels[i].ErrorFlags |= CHN_OVERCURRENT_LIMIT;
        }
        else
        {
          // No conditions found. Clear flag
          Channels[i].ErrorFlags = 0;
        }

        Channels[i].CurrentValue = amps;
      }
      else
      {
        updatePWMDutyCycle(i, 0);
        Channels[i].CurrentValue = 0.0;
      }
      break;
    case DIG:
      if (Channels[i].Enabled)
      {
        // Read analog values first
        if (Channels[i].Enabled)
        {
          int sum = 0;
          uint8_t total = 0;
          for (int j = 0; j < ANALOG_READ_SAMPLES; j++)
          {
            sum += analogRead(Channels[i].CurrentSensePin);
            total++;
          }
          float analogMean = 0.0f;
          if (total)
          {
            analogMean = sum / total;
            Channels[i].AnalogRaw = analogMean;
          }

          float milliVolts = (analogMean / (float)ADCres) * V_REF;

          float I_IS = milliVolts / R_IS;
          Channels[i].CurrentValue = k_ILIS * I_IS;
        }
        updatePWMDutyCycle(i, 255);
      }
      else
      {
        updatePWMDutyCycle(i, 0);
      }
      break;
    default:
      updatePWMDutyCycle(i, 0);
      Channels[i].CurrentValue = 0.0;
      break;
    }
  }
}

void OutputsOff()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    updatePWMDutyCycle(i, 0);
    Channels[i].CurrentValue = 0.0;
    Channels[i].ErrorFlags = 0;
  }
}