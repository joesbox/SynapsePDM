/*  Battery.h Battery charging variables, functions and data handling.
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

#ifndef Battery_H
#define Battery_H

#include <Globals.h>
#include <SparkFunBQ27441.h>

/// @brief Battery capacity in mAh
const uint16_t BATTERY_CAPACITY = 2000;

/// @brief Lowest operational voltage in mV
const uint16_t TERMINATE_VOLTAGE = 3000;

/// @brief Termination current. T4056 uses a C/10 Charge Termination. With a 1.2K current setting resistor, this would be 100mA.
const uint16_t TAPER_CURRENT = 100;

/// @brief Battery state of charge (%)
extern int SOC;

/// @brief Battery state of health (%)
extern int SOH;

/// @brief Initialises battery SOC reading
void InitialiseBattery();

/// @brief Debug battery status
void printBatteryStats();

/// @brief Manages the backup battery
void ManageBattery();

#endif