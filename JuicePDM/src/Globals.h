/*  Globals.h Global variables, definitions and functions.
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

#ifndef Globals_H
#define Globals_H

#include <Arduino.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>
#include <ChannelConfig.h>
#include <System.h>
#include <IMU.h>
#include <SPI.h>
#include <Storage.h>

// Firmware version
#define FW_VER "0.1.1"

// Build date
#define BUILD_DATE __DATE__ " " __TIME__

// Number of hardware output channels
#define NUM_CHANNELS 14

// Number of digital input channels
#define NUM_DI_CHANNELS 8

// Number of analogue input channels
#define NUM_ANA_CHANNELS 8

// Delay (microseconds) before making an analog reading
// TODO: measure actual turn on delay and adjust this value
#define ANALOG_DELAY 100

// Maximum PWM duty accounting for turn off delay. Above this value, a PWM channel will be set to 100% duty
// BTS50010-1LUA max turn off time is 220µs. Default PWM frequency is 200Hz (5000µs period). Therefore, max. PWM duty is limited to about 95%, 4750µs
#define MAX_DUTY 95

// Min duty accounting for turn on delay. Above this value, a PWM channel will be set to 0% duty.
// BTS50010-1LUA max turn on delay is 190µs. Default PWM frequency is 200Hz (5000µs period). Therefore, min. PWM duty is limited to about 10% (500µs, 190µs turn-on delay + analog read)
#define MIN_DUTY 10

// IS current fault threshold voltage. Aobve this threshold, the channel is either open circuit, short circuit or over temperature
#define FAULT_THRESHOLD 2.00

// Microsecond representation of a CPU tick
#define CPU_TICK_MICROS (1E6 / F_CPU)

// Maximum per-channel current supported by hardware. No channel can exceed this limit.
#define CURRENT_MAX 17.0

// Sytem current max (total). Should never reach this as each channel is independantly monitored
#define SYSTEM_CURRENT_MAX 78.0

// Default current sense ratio as specified by the BTS50010 datasheet
#define DEFAULT_DK_VALUE 38000

// Maximum log file size in bytes
#define MAX_LOGFILE_SIZE 100000

// Main task timer intervals (milliseconds)
#define TASK_0_INTERVAL 10
#define TASK_1_INTERVAL 50
#define TASK_2_INTERVAL 80
#define TASK_3_INTERVAL 100
#define TASK_4_INTERVAL 60000
#define GPS_INTERVAL 100

#define DEBUG_INTERVAL 1000
#define DEBUG_PIN PA15

// Watchdog timer interval
#define WATCHDOG_INTERVAL 2500

// Unused pin that can be used to debug analog read timings which are critical to obtaining correct current measurements on PWM channels
#define ANALOG_READ_DEBUG_PIN 20

// Debug flag
#define DEBUG

// Battery measurement analog input pin
#define VBATT_ANALOG_PIN PC4

// Nominal battery voltage
#define VBATT_NOMINAL 13.8

// CAN bus termination resistor enable pin
#define CAN_BUS_RESISTOR_ENABLE 6

// Battery voltage threshold at which power loss is immenent and logging should be stopped
#define LOGGING_VBATT_THRESHOLD 9.0

// Maximum permissible system temperature
#define SYSTEM_TEMP_LIMIT 80.0

// System error bitmasks
#define OVERCURRENT 0x01
#define OVERTEMP 0x02
#define UNDERVOLTGAGE 0x04
#define CRC_CHECK_FAILED 0x08
#define SDCARD_ERROR 0x10

// Channel error bitmasks
#define CHN_OVERCURRENT_RANGE 0x01
#define CHN_OVERCURRENT_LIMIT 0x02
#define CHN_UNDERCURRENT_RANGE 0x04
#define OVERTEMP_GNDSHORT 0x08
#define WATCHDOG_TIMEOUT 0x10
#define IS_FAULT 0x20

// ECU CAN address
#define ECU_ADDR 0x800

// Ignition input (KL15)
#define IGN_INPUT PE2

// IMU Pins
#define IMU_INT1 PE4
#define IMU_INT2 PE5

// Default wake window for IMU checks
#define DEFAULT_WW 5000

// Default log frequecy of 10Hz
#define DEFAULT_LOG_FREQUENCY 10

// Default number of log lines. 36000 = 1 hour @ 10Hz
#define DEFAULT_LOG_LINES 36000

// Number of logs to keep on the SD card
#define NUMBER_LOGS 10

// Power enable pins
#define PWR_EN_5V PE7
#define PWR_EN_3V3 PF11

// Power states
#define RUN 0
#define PREPARE_SLEEP 1
#define SLEEPING 2
#define IGNITION_WAKE 3
#define IMU_WAKE 4
#define IMU_WAKE_WINDOW 5

// SPI 2 Pins
#define PICO PB15
#define POCI PB14
#define SCK2 PB13
#define CS1 PB12
#define CS2 PB11

// LCD pins
#define TFT_RST PD8
#define TFT_DC PD9
#define TFT_BL PB10

// GSM Module Pins
#define SIM_PWR PC6
#define SIM_RST PC7
#define SIM_FLIGHT PB8

// SPI 2
extern SPIClass SPI_2;

extern DMA_HandleTypeDef hdma_tx;

/// @brief Analogue input config structure
struct __attribute__((packed)) AnalogueInputs
{
    uint8_t InputPin;    // Input pin
    uint8_t PullUpPin;   // Pull-up enable pin
    uint8_t PullDownPin; // Pull-down enable pin
    bool PullUpEnable;   // Pull-up enable flag
    bool PullDownEnable; // Pull-down enable flag
};

// Channel digital input pins (defaults)
const uint8_t DIchannelInputPins[NUM_DI_CHANNELS] = {PE15, PE14, PE13, PE12, PE11, PE10, PE9, PE8};

// Channel analogue input pins (defaults)
const uint8_t ANAchannelInputPins[NUM_ANA_CHANNELS] = {PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10};

// Channel analogue input pull-up pins (defaults)
const uint8_t ANAchannelInputPullUps[NUM_ANA_CHANNELS] = {PD3, PD5, PD7, PG12, PG15, PB4, PB9, PE1};

// Channel analogue input pull-down pins (defaults)
const uint8_t ANAchannelInputPullDowns[NUM_ANA_CHANNELS] = {PD4, PD6, PG11, PG13, PG14, PB3, PB5, PE0};

// Channel digital output pins
const uint8_t channelOutputPins[NUM_CHANNELS] = {PG10, PG9, PG6, PG5, PG4, PG3, PG2, PF15, PF14, PF13, PF12, PF2, PF1, PF0};

// Channel analog current sense pins
const uint8_t channelCurrentSensePins[NUM_CHANNELS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PB1, PB0, PA7, PC3, PC2, PC1, PC0};

// Timers for main tasks
extern uint32_t task0Timer;
extern uint32_t task1Timer;
extern uint32_t task2Timer;
extern uint32_t task3Timer;
extern uint32_t task4Timer;
extern uint32_t debugTimer;
extern uint32_t imuWWtimer;
extern uint32_t GPStimer;
extern uint32_t LogTimer;

// HSD Output channels
extern ChannelConfig Channels[NUM_CHANNELS];

// Channel configurations
extern AnalogueInputs AnalogueIns[NUM_ANA_CHANNELS];

/// @brief  Channel config union for reading and writing from and to EEPROM storage
union ChannelConfigUnion
{
  ChannelConfig data[NUM_CHANNELS];
  byte dataBytes[sizeof(Channels)];
};

/// @brief Config storage union for channel data
extern ChannelConfigUnion ChannelConfigData;

/// @brief Initialise channel data to known defaults
void InitialiseChannelData();

/// @brief Initialise system data to known 
void InitialiseSystemData();

#endif