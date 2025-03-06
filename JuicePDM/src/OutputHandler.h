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

// Max frequency specified by the BTS50010A is 200Hz. This 8-bit PWM period represents aaprox. 20µs per count which translates to a PWM frequency of approximately 195Hz
#define PWM_COUNT_INTERVAL 20

// How many current sense samples to collect to calculate a mean
#define ANALOG_READ_SAMPLES 10

// BTS50010-1LUA max turn on delay is 190µs (80%), max turn off delay is 200µs (20%). The effective usable 8-bit PWM range is 21 to 235
#define MIN_PWM 21
#define MAX_PWM 235

// Polynomial terms used to calculate current
#define PTERM1 9.0829
#define PTERM2 -24.874
#define PTERM3 26.468
#define PTERM4 -9.5747
#define PCONST 1.2549

// Interval timer used to control PWM outputs and analog read back
//extern IntervalTimer myTimer;

// Toggle error LED
extern uint8_t toggle[NUM_CHANNELS];

// Setup interrupts and analog read timers
void InitialiseOutputs();

// Set PWM or digital outputs
void UpdateOutputs();

/// @brief Interval timer callback to control PWM outputs
void OutputTimer();

/// @brief Turn all outputs off
void OutputsOff();

#endif
