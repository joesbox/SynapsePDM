
/*  OutputHandler.cpp Output handler deals with channel output control.
    Specifically applies to the Infineon BTS50010 High-Side Driver
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

#include "OutputHandler.h"
#include "SerialComms.h"

// Pins to update
const uint16_t GPIOG_PINS[] = {GPIO_PIN_10, GPIO_PIN_9, GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_3, GPIO_PIN_2}; // Outputs 1 to 7
const uint8_t NUM_PINS_G = sizeof(GPIOG_PINS) / sizeof(GPIOG_PINS[0]);
const uint16_t GPIOG_ALL_PINS = GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2;

const uint16_t GPIOF_PINS[] = {GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_12, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0}; // Outputs 8 to 14
const uint8_t NUM_PINS_F = sizeof(GPIOF_PINS) / sizeof(GPIOF_PINS[0]);
const uint16_t GPIOF_ALL_PINS = GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13 | GPIO_PIN_12 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0;

// DMA buffer for multiple pins
uint32_t pwmBufferG[100] = {0};
uint32_t pwmBufferF[100] = {0};

// Independent duty cycle tracking
uint8_t dutyCycles[14] = {0};

// Timer and DMA handles
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim1;

// DMA handles at file scope so sleep can stop the active streams cleanly
static DMA_HandleTypeDef hdma_tim8_up;
static DMA_HandleTypeDef hdma_tim1_up;

// Soft start tracking (must be after NUM_CHANNELS is defined in Globals.h)

static uint32_t softStartTimers[NUM_CHANNELS] = {0};
static bool softStartActive[NUM_CHANNELS] = {false};
static uint32_t softStopTimers[NUM_CHANNELS] = {0};
static bool softStopActive[NUM_CHANNELS] = {false};
static uint8_t softStopStartDuty[NUM_CHANNELS] = {0};
static bool previousEnabled[NUM_CHANNELS] = {false};

volatile uint8_t analogCounter;
uint analogValues[NUM_CHANNELS][ANALOG_READ_SAMPLES];

// Channel number used to identify associated channel
int channelNum;

// Track if retries are pending
bool retriesPending[NUM_CHANNELS] = {false};

// Retry timers
unsigned long retryTimers[NUM_CHANNELS] = {0};

// Channel lock status
bool channelLocked[NUM_CHANNELS] = {false};

// per-channel counters
uint8_t retryCount[NUM_CHANNELS] = {0};

// Per-channel EMA filter state for PWM current readback
static float pwmCurrentFiltered[NUM_CHANNELS] = {0.0f};
static bool pwmCurrentFilterPrimed[NUM_CHANNELS] = {false};
static uint8_t digitalTrySampleCount[NUM_CHANNELS] = {0};
static float digitalTryCurrentSum[NUM_CHANNELS] = {0.0f};
static bool outputsInhibited = false;

bool IsChannelThermallyShed(uint8_t channelIndex)
{
  if (channelIndex >= NUM_CHANNELS)
  {
    return false;
  }

  if (ActiveThermalProtectionStage == THERMAL_PROTECTION_NONE)
  {
    return false;
  }

  ChannelPriority priority = GetChannelPriority(Channels[channelIndex].Category);
  if (ActiveThermalProtectionStage == THERMAL_PROTECTION_WARNING)
  {
    return priority == CHANNEL_PRIORITY_LOW;
  }

  return priority != CHANNEL_PRIORITY_CRITICAL;
}

bool IsChannelEffectivelyEnabled(uint8_t channelIndex)
{
  if (channelIndex >= NUM_CHANNELS)
  {
    return false;
  }

  return IsChannelRuntimeEnabled(channelIndex) && !outputsInhibited && !IsChannelThermallyShed(channelIndex);
}

static bool ApplySoftStartRamp(uint8_t channelIndex, bool triggerRamp, int targetDuty, int &rampDuty)
{
  if (Channels[channelIndex].SoftStart && Channels[channelIndex].SoftStartTime > 0)
  {
    if (triggerRamp)
    {
      softStartActive[channelIndex] = true;
      softStartTimers[channelIndex] = millis();
    }

    if (softStartActive[channelIndex])
    {
      uint32_t elapsed = millis() - softStartTimers[channelIndex];
      if (elapsed < Channels[channelIndex].SoftStartTime)
      {
        float ramp = (float)elapsed / (float)Channels[channelIndex].SoftStartTime;
        rampDuty = (int)(ramp * targetDuty);
        if (rampDuty > 100)
        {
          rampDuty = 100;
        }
        if (rampDuty < 0)
        {
          rampDuty = 0;
        }
        return true;
      }

      softStartActive[channelIndex] = false;
    }
  }
  else
  {
    softStartActive[channelIndex] = false;
  }

  rampDuty = targetDuty;
  return false;
}

static bool ApplySoftStopRamp(uint8_t channelIndex, bool triggerRamp, int &rampDuty)
{
  if (triggerRamp)
  {
    softStartActive[channelIndex] = false;

    if (Channels[channelIndex].SoftStop && Channels[channelIndex].SoftStopTime > 0 && dutyCycles[channelIndex] > 0)
    {
      softStopActive[channelIndex] = true;
      softStopTimers[channelIndex] = millis();
      softStopStartDuty[channelIndex] = dutyCycles[channelIndex];
    }
    else
    {
      softStopActive[channelIndex] = false;
      softStopStartDuty[channelIndex] = 0;
    }
  }

  if (softStopActive[channelIndex])
  {
    uint32_t elapsed = millis() - softStopTimers[channelIndex];
    if (elapsed < Channels[channelIndex].SoftStopTime)
    {
      float ramp = 1.0f - ((float)elapsed / (float)Channels[channelIndex].SoftStopTime);
      rampDuty = (int)(ramp * softStopStartDuty[channelIndex]);
      if (rampDuty > 100)
      {
        rampDuty = 100;
      }
      if (rampDuty < 0)
      {
        rampDuty = 0;
      }
      return true;
    }

    softStopActive[channelIndex] = false;
    softStopStartDuty[channelIndex] = 0;
  }

  rampDuty = 0;
  return false;
}

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

void SleepOutputs()
{
  // Stop PWM DMA activity before STOP mode; otherwise repeated wake cycles can
  // leave the timer/DMA pair active and block deep sleep entry.
  __HAL_TIM_DISABLE_DMA(&htim8, TIM_DMA_UPDATE);
  __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
  HAL_TIM_Base_Stop(&htim8);
  HAL_TIM_Base_Stop(&htim1);
  HAL_DMA_Abort(&hdma_tim8_up);
  HAL_DMA_Abort(&hdma_tim1_up);

  // Explicitly clear the GPIO output latches once DMA is stopped so no channel
  // can remain physically high when we drop into sleep.
  HAL_GPIO_WritePin(GPIOG, GPIOG_ALL_PINS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOF, GPIOF_ALL_PINS, GPIO_PIN_RESET);

  __HAL_RCC_GPIOB_CLK_SLEEP_DISABLE();
  __HAL_RCC_GPIOC_CLK_SLEEP_DISABLE();
  __HAL_RCC_GPIOD_CLK_SLEEP_DISABLE();
  // Keep GPIOE clocked in STOP mode because the wake sources live on PE2 and PE4.
  __HAL_RCC_GPIOE_CLK_SLEEP_ENABLE();
  __HAL_RCC_GPIOF_CLK_SLEEP_DISABLE();
  __HAL_RCC_GPIOG_CLK_SLEEP_DISABLE();

  __HAL_RCC_DMA1_CLK_SLEEP_DISABLE();
  __HAL_RCC_DMA2_CLK_SLEEP_DISABLE();

  __HAL_RCC_TIM1_CLK_SLEEP_DISABLE();
  __HAL_RCC_TIM8_CLK_SLEEP_DISABLE();
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
  hdma_tim8_up.Instance = DMA2_Stream1;
  hdma_tim8_up.Init.Channel = DMA_CHANNEL_7;
  hdma_tim8_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim8_up.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim8_up.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim8_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim8_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim8_up.Init.Mode = DMA_CIRCULAR;
  hdma_tim8_up.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim8_up.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  HAL_DMA_Init(&hdma_tim8_up);
  htim8.hdma[TIM_DMA_ID_UPDATE] = &hdma_tim8_up;

  HAL_DMA_Start(&hdma_tim8_up, (uint32_t)pwmBufferG, (uint32_t)&GPIOG->BSRR, 100);

  // Second DMA (Stream 5, Channel 6)
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
  htim1.hdma[TIM_DMA_ID_UPDATE] = &hdma_tim1_up;

  HAL_DMA_Start(&hdma_tim1_up, (uint32_t)pwmBufferF, (uint32_t)&GPIOF->BSRR, 100);
}

void configureTimer()
{
  __HAL_RCC_TIM8_CLK_ENABLE();

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 84 - 1; // TIM8 clock 168MHz / 84 = 2MHz timer tick
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 133 - 1; // 2MHz / 133 ~= 15.0kHz update => ~150Hz PWM (100-step buffer)
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  HAL_TIM_Base_Init(&htim8);
  HAL_TIM_Base_Start(&htim8);

  __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_UPDATE);

  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84 - 1; // TIM1 clock 168MHz / 84 = 2MHz timer tick
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 133 - 1; // 2MHz / 133 ~= 15.0kHz update => ~150Hz PWM (100-step buffer)
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
  if (outputsInhibited)
  {
    OutputsOff();
    return;
  }

  const int currentSenseSamples = IsCortexConfigSaveActive() ? 1 : ANALOG_READ_SAMPLES;

  // Check the type of channel we're dealing with (digital or PWM) and handle output accordingly
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    bool runtimeEnabled = IsChannelRuntimeEnabled(i);
    bool thermallyShed = runtimeEnabled && IsChannelThermallyShed(i);

    // Detect rising edge of enable
    bool risingEdge = runtimeEnabled && !previousEnabled[i];
    bool fallingEdge = !runtimeEnabled && previousEnabled[i];
    previousEnabled[i] = runtimeEnabled;

    if (!thermallyShed)
    {
      ChannelRuntime[i].ErrorFlags &= (uint8_t)~CHN_TEMP_SHUTDOWN;
    }

    // Only perform current measurement and fault diagnosis if channel is enabled

    if (runtimeEnabled)
    {
    }
    else
    {
      ChannelRuntime[i].CurrentValue = 0.0;
      pwmCurrentFiltered[i] = 0.0f;
      pwmCurrentFilterPrimed[i] = false;
    }

    if (thermallyShed)
    {
      updatePWMDutyCycle(i, 0);
      ChannelRuntime[i].CurrentValue = 0.0f;
      ChannelRuntime[i].ErrorFlags |= CHN_TEMP_SHUTDOWN;
      digitalTrySampleCount[i] = 0;
      digitalTryCurrentSum[i] = 0.0f;
      retriesPending[i] = false;
      softStartActive[i] = false;
      softStopActive[i] = false;
      softStopStartDuty[i] = 0;
      pwmCurrentFiltered[i] = 0.0f;
      pwmCurrentFilterPrimed[i] = false;
      continue;
    }

    switch (Channels[i].ChanType)
    {
    case DIG_PWM:
    case ANA_PWM:
      if (runtimeEnabled)
      {
        softStopActive[i] = false;
        softStopStartDuty[i] = 0;
        bool criticalFault = false;
        int sum = 0;
        uint8_t total = 0;
        for (int j = 0; j < currentSenseSamples; j++)
        {
          sum += analogRead(Channels[i].CurrentSensePin);
          total++;
        }
        float analogMean = 0.0f;
        if (total)
        {
          analogMean = sum / total;
          ChannelRuntime[i].AnalogRaw = analogMean;
        }

        float isVoltage = (ChannelRuntime[i].AnalogRaw / ADCres) * V_REF;
        float measuredAmps = (PWM_M * ChannelRuntime[i].AnalogRaw) + PWM_C;

        if (ChannelRuntime[i].AnalogRaw < 5)
        {
          measuredAmps = 0.0;
        }

        float squared = (VBATT_NOMINAL / SystemRuntimeParams.VBatt) * (VBATT_NOMINAL / SystemRuntimeParams.VBatt);
        int pwmActual = round(Channels[i].PWMSetDuty * squared);
        if (pwmActual > 100)
        {
          pwmActual = 100;
        }
        if (pwmActual < 0)
        {
          pwmActual = 0;
        }

        float amps = measuredAmps;
        if (pwmActual <= 0)
        {
          // Keep true zero when duty is effectively off.
          pwmCurrentFiltered[i] = 0.0f;
          pwmCurrentFilterPrimed[i] = false;
          amps = 0.0f;
        }
        else
        {
          if (!pwmCurrentFilterPrimed[i])
          {
            pwmCurrentFiltered[i] = measuredAmps;
            pwmCurrentFilterPrimed[i] = true;
          }
          else
          {
            pwmCurrentFiltered[i] += PWM_CURRENT_FILTER_ALPHA * (measuredAmps - pwmCurrentFiltered[i]);
          }

          if ((measuredAmps <= 0.0f) && (pwmCurrentFiltered[i] < PWM_CURRENT_ZERO_SNAP_AMPS))
          {
            pwmCurrentFiltered[i] = 0.0f;
          }
          amps = pwmCurrentFiltered[i];
        }

        bool inrushPeriod = (millis() - enabledTimers[i]) <= (unsigned long)(Channels[i].InrushDelay);

        float overCurrentThreshold = Channels[i].CurrentThresholdHigh;
        if (inrushPeriod)
        {
          overCurrentThreshold = Channels[i].InrushCurrentThreshold;
        }

        if (isVoltage > FAULT_THRESHOLD)
        {
          ChannelRuntime[i].ErrorFlags |= IS_FAULT;
          criticalFault = true;
        }
        else if (amps > overCurrentThreshold)
        {
          ChannelRuntime[i].ErrorFlags |= CHN_OVERCURRENT;
          criticalFault = true;
        }
        else if (!inrushPeriod && amps < Channels[i].CurrentThresholdLow)
        {
          ChannelRuntime[i].ErrorFlags |= CHN_UNDERCURRENT;
        }
        else
        {
          if (!channelLocked[i])
          {
            ChannelRuntime[i].ErrorFlags = 0;
          }
        }

        ChannelRuntime[i].CurrentValue = amps;

        int targetDuty = pwmActual;
        int rampDuty = targetDuty;
        bool ramping = false;
        bool anaPwmDutyRiseFromZero = (Channels[i].ChanType == ANA_PWM) && !softStartActive[i] && (dutyCycles[i] == 0) && (targetDuty > 0);
        ramping = ApplySoftStartRamp(i, risingEdge || anaPwmDutyRiseFromZero, targetDuty, rampDuty);

        if (rampDuty > 100)
        {
          rampDuty = 100;
        }
        if (rampDuty < 0)
        {
          rampDuty = 0;
        }

        if (ramping)
        {
          if (criticalFault)
          {
            updatePWMDutyCycle(i, 0);
            softStartActive[i] = false;
            pwmCurrentFiltered[i] = 0.0f;
            pwmCurrentFilterPrimed[i] = false;
            break;
          }
          updatePWMDutyCycle(i, rampDuty);
          continue;
        }
        updatePWMDutyCycle(i, targetDuty);
      }
      else
      {
        int rampDuty = 0;
        if (ApplySoftStopRamp(i, fallingEdge, rampDuty))
        {
          updatePWMDutyCycle(i, rampDuty);
        }
        else
        {
          updatePWMDutyCycle(i, 0);
          pwmCurrentFiltered[i] = 0.0f;
          pwmCurrentFilterPrimed[i] = false;
        }
      }
      break;
    case DIG:
    case DIG_INTERMITTENT:
    case ANA:
    case CAN_DIGITAL:
      if (runtimeEnabled)
      {
        softStopActive[i] = false;
        softStopStartDuty[i] = 0;
        int sum = 0;
        uint8_t total = 0;
        for (int j = 0; j < currentSenseSamples; j++)
        {
          sum += analogRead(Channels[i].CurrentSensePin);
          total++;
        }
        float analogMean = 0.0f;
        if (total)
        {
          analogMean = sum / total;
          ChannelRuntime[i].AnalogRaw = analogMean;
        }

        float milliVolts = (analogMean / (float)ADCres) * V_REF;
        float I_IS = milliVolts / R_IS;
        ChannelRuntime[i].CurrentValue = Channels[i].CurrentSenseKILIS * I_IS;
        if (ChannelRuntime[i].AnalogRaw < 5)
        {
          ChannelRuntime[i].CurrentValue = 0.0;
        }

        int targetDuty = 100;
        int rampDuty = targetDuty;
        bool ramping = false;
        ramping = ApplySoftStartRamp(i, risingEdge, targetDuty, rampDuty);
        if (rampDuty > 100)
        {
          rampDuty = 100;
        }
        if (rampDuty < 0)
        {
          rampDuty = 0;
        }

        if (ramping)
        {
          updatePWMDutyCycle(i, rampDuty);
          // Continue ramping (do not overwrite current value)
          continue;
        }

        if (risingEdge)
        {
          digitalTrySampleCount[i] = 0;
          digitalTryCurrentSum[i] = 0.0f;

          if (!channelLocked[i])
          {
            updatePWMDutyCycle(i, targetDuty);
          }
        }

        // Determine which threshold to use based on inrush delay
        bool inrushPeriod = (millis() - enabledTimers[i]) <= (unsigned long)(Channels[i].InrushDelay);
        digitalTryCurrentSum[i] += ChannelRuntime[i].CurrentValue;
        digitalTrySampleCount[i]++;
        if (digitalTrySampleCount[i] >= 3)
        {
          float avgCurrent = digitalTryCurrentSum[i] / digitalTrySampleCount[i];
          digitalTrySampleCount[i] = 0;
          digitalTryCurrentSum[i] = 0.0f;
          if (!channelLocked[i])
          {
            ChannelRuntime[i].ErrorFlags = 0;
          }
          float overCurrentThreshold = inrushPeriod ? Channels[i].InrushCurrentThreshold : Channels[i].CurrentThresholdHigh;
          if (avgCurrent > overCurrentThreshold)
          {
            ChannelRuntime[i].ErrorFlags |= CHN_OVERCURRENT;
          }
          else if (!inrushPeriod && avgCurrent < Channels[i].CurrentThresholdLow)
          {
            ChannelRuntime[i].ErrorFlags |= CHN_UNDERCURRENT;
          }
          if (ChannelRuntime[i].ErrorFlags == 0)
          {
            if (!channelLocked[i])
            {
              updatePWMDutyCycle(i, 100);
            }
            else
            {
              updatePWMDutyCycle(i, 0);
            }
            retriesPending[i] = false;
          }
          else
          {
            if (!channelLocked[i])
            {
              retryCount[i]++;
              updatePWMDutyCycle(i, 0);
              if (retryCount[i] > Channels[i].RetryCount)
              {
                channelLocked[i] = true;
                ChannelRuntime[i].ErrorFlags |= RETRY_LOCKOUT;
                updatePWMDutyCycle(i, 0);
              }
            }
            else
            {
              updatePWMDutyCycle(i, 0);
            }
          }
        }
      }
      else
      {
        int rampDuty = 0;
        if (ApplySoftStopRamp(i, fallingEdge, rampDuty))
        {
          updatePWMDutyCycle(i, rampDuty);
        }
        else
        {
          updatePWMDutyCycle(i, 0);
        }
        retriesPending[i] = false;
        retryCount[i] = 0;
        channelLocked[i] = false;
        digitalTrySampleCount[i] = 0;
        digitalTryCurrentSum[i] = 0.0f;
        ChannelRuntime[i].CurrentValue = 0.0;
        pwmCurrentFiltered[i] = 0.0f;
        pwmCurrentFilterPrimed[i] = false;
      }
      break;
    default:
      updatePWMDutyCycle(i, 0);
      ChannelRuntime[i].CurrentValue = 0.0;
      pwmCurrentFiltered[i] = 0.0f;
      pwmCurrentFilterPrimed[i] = false;
      break;
    }
  }
}

void OutputsOff()
{
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    updatePWMDutyCycle(i, 0);
    previousEnabled[i] = false;
    enabledFlags[i] = false;
    ChannelRuntime[i].Enabled = false;
    ChannelRuntime[i].CurrentValue = 0.0;
    ChannelRuntime[i].ErrorFlags = 0;
    retriesPending[i] = false;
    retryCount[i] = 0;
    channelLocked[i] = false;
    digitalTrySampleCount[i] = 0;
    digitalTryCurrentSum[i] = 0.0f;
    softStartActive[i] = false;
    softStopActive[i] = false;
    softStopStartDuty[i] = 0;
    pwmCurrentFiltered[i] = 0.0f;
    pwmCurrentFilterPrimed[i] = false;
  }
}

void SetOutputsInhibited(bool inhibited)
{
  outputsInhibited = inhibited;
  invalidateDisplay = true;

  if (outputsInhibited)
  {
    OutputsOff();
  }
}

bool AreOutputsInhibited()
{
  return outputsInhibited;
}