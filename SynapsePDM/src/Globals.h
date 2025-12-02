/*  Globals.h Global variables, definitions and functions.
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

#ifndef Globals_H
#define Globals_H

#include <Arduino.h>
#include <Wire.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>
#include <ChannelConfig.h>
#include <System.h>
#include <IMU.h>
#include <SPI.h>
#include <Storage.h>

// Firmware version
#define FW_VER "v0.2"

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

// IS current fault threshold voltage. Above this threshold, the channel is either open circuit, short circuit or over temperature
#define FAULT_THRESHOLD 2.00

// Microsecond representation of a CPU tick
#define CPU_TICK_MICROS (1E6 / F_CPU)

// Maximum per-channel current supported by hardware. No channel can exceed this limit.
#define CURRENT_MAX 17.0

// Channel inrush delay (milliseconds)
#define INRUSH_DELAY 500

// Maximum inrush delay (milliseconds). This has to be balanced with what the wiring harness can support
#define INRUSH_MAX 2000

// Sytem current max (total). Should never reach this as each channel is independantly monitored
#define SYSTEM_CURRENT_MAX 240

// Default current sense ratio as specified by the BTS50010 datasheet
#define DEFAULT_DK_VALUE 38000

// Main task timer intervals (milliseconds)
#define DISPLAY_INTERVAL 50
#define COMMS_INTERVAL 100
#define BATTERY_INTERVAL 60000
#define LOG_INTERVAL 100
#define GPS_INTERVAL 1000
#define BL_FADE_INTRVAL 0

#define DEBUG_INTERVAL 1000

// Watchdog timer interval (microseconds)
#define WATCHDOG_INTERVAL 2000000

// Unused pin that can be used to debug analog read timings which are critical to obtaining correct current measurements on PWM channels
#define ANALOG_READ_DEBUG_PIN 20

// Debug flag
// #define DEBUG
#define DEBUG_PIN PA15

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
#define OVERCURRENT 0x0001
#define OVERTEMP 0x0002
#define UNDERVOLTAGE 0x0004
#define CRC_CHECK_FAILED 0x0008
#define SDCARD_ERROR 0x0010
#define PC_COMMS_CHECKSUM_ERROR 0x0020
#define GPS_ERROR 0x0040

// Channel error bitmasks
#define CHN_OVERCURRENT_RANGE 0x01
#define CHN_UNDERCURRENT_RANGE 0x02
#define IS_FAULT 0x04
#define RETRY_LOCKOUT 0x08

// ECU CAN addresses
#define CHAN_CAN_ID 0x800
#define SYS_CAN_ID 0x801
#define CONF_CAN_ID 0x802

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

// Charging pins
#define CHARGE_EN PG0
#define BATT_INT PG1

// Battery states
#define COMMSOK 0
#define BATTERY_CHARGED 1
#define BATTERY_CHARGING 2
#define BATTERY_LOW 3
#define BATTERY_CRITICAL 4

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
//#define TFT_RST PD8
//#define TFT_DC PD9
//#define TFT_CS PB11
#define TFT_BL PB10

// GSM Module Pins
#define SIM_PWR PC6
#define SIM_RST PC7
#define SIM_FLIGHT PB8

#define COMMS_TIMEOUT 5000

/// @brief Output enabled flags
extern bool enabledFlags[NUM_CHANNELS];

/// @brief Output enabled timers
extern unsigned long enabledTimers[NUM_CHANNELS];

// SPI 2
extern SPIClass SPI_2;

/// @brief PC connection status. 0 = disconnected, 1 = connected, 2 = Checksum fail
extern uint8_t connectionStatus;
extern int recBytesRead;

/// @brief Flag to denote if the background has been drawn
extern bool backgroundDrawn;

/// @brief Analogue input config structure
struct __attribute__((packed)) AnalogueInputs
{
  uint8_t InputPin;    // Input pin
  uint8_t PullUpPin;   // Pull-up enable pin
  uint8_t PullDownPin; // Pull-down enable pin
  bool PullUpEnable;   // Pull-up enable flag
  bool PullDownEnable; // Pull-down enable flag
  bool IsDigital;      // True if the input is to be treated as a digital input
  bool IsThreshold;    // True if the input is to be treated as a thresholded input or PWM input (false = scaled PWM). Only applies to analogue inputs
  float OnThreshold;   // On threshold (Voltage)
  float OffThreshold;  // Off threshold (Voltage)
  float ScaleMin;      // Minimum scale value (Used for PWM scaled inputs)
  float ScaleMax;      // Maximum scale value (Used for PWM scaled inputs)
  uint8_t PWMMin;      // Minimum PWM value (0-100%)
  uint8_t PWMMax;        // Maximum PWM value (0-100%)
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
extern uint32_t imuWWtimer;
extern uint32_t DisplayTimer;
extern uint32_t CommsTimer;
extern uint32_t BattTimer;
extern uint32_t LogTimer;
extern uint32_t GPSTimer;
extern uint32_t BLTimer;
extern int blLevel;

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

extern uint8_t RTCyear;
extern uint8_t RTCmonth;
extern uint8_t RTCday;
extern uint8_t RTChour;
extern uint8_t RTCminute;
extern uint8_t RTCsecond;

/// @brief Initialise channel data to known defaults
void InitialiseChannelData();

/// @brief Initialise system data to known
void InitialiseSystemData();

#endif