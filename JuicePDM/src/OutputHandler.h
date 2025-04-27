/*  OutputHandler.h Output handler deals with channel output control.
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

#ifndef OutputHandler_H
#define OutputHandler_H

#include <Arduino.h>
#include <Globals.h>
#include <System.h>

// How many current sense samples to collect to calculate a mean
#define ANALOG_READ_SAMPLES 100

#define PWM_M 0.0168F
#define PWM_C 0.0062F

#define V_REF 3.294F  // Reference voltage for ADC
#define R_IS 1000.0F  // Sense resistor value in ohms (1kΩ)
#define ADCres 4095 // 12-bit ADC resolution

#define k_ILIS 18407.72F // Current sense ratio

/// @brief Setup interrupts and analog read timers
void InitialiseOutputs();

/// @brief Setup GPIO for outputs. Push-pull, no pullups.
void setupGPIO();

/// @brief Configure DMA. Memory -> peripheral to set BSRR.
void configureDMA();

/// @brief Configure two timers to trigger DMA.
void configureTimer();

/// @brief Update duty cycle
/// @param pinIndex Pin index
/// @param dutyCycle Duty cycle in percentage (0 - 100)
void updatePWMDutyCycle(uint8_t pinIndex, uint8_t dutyCycle);

/// @brief Set PWM or digital outputs
void UpdateOutputs();

/// @brief Turn all outputs off
void OutputsOff();

/// @brief Output timer handler
void OutputTimer();

#endif
