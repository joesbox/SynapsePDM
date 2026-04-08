/*  IMU.h IMU functions.
    Copyright (c) 2025 Joe Mann.  All right reserved.
    
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

#ifndef IMU_H
#define IMU_H

#define BMM350_ZERO_RECOVERY_THRESHOLD 3
#define BMM350_RECOVERY_RETRY_MS 1000UL
#define BMM350_WAKE_RECOVERY_DELAY_MS 250UL

/// @brief Initialise the IMU
void InitialiseIMU();

/// @brief Reinitialise the shared IMU I2C devices after wake
void ReinitialiseIMUAfterWake();

/// @brief Reinitialise the BMM350 after the switched 3.3V rail returns
void ReinitialiseMagnetometer();

/// @brief Put the BMI270 into a low-power sleep profile while optionally keeping motion wake active
void SleepIMU(bool allowMotionWake);

/// @brief Read the IMU
void ReadIMU();

/// @brief Enable the interrupt for sleep mode
void EnableMotionDetect();

/// @brief Disable the interrupt for run mode
void DisableMotionDetect();

/// @brief IMU initialisation status
extern bool IMUOK;

/// @brief X-axis acceleration in Gs
extern float accelX;

/// @brief Y-axis acceleration in Gs
extern float accelY;

/// @brief Z-axis acceleration in Gs
extern float accelZ;

/// @brief X-axis rotation in deg/sec
extern float gyroX;

/// @brief Y-axis rotation in deg/sec
extern float gyroY;

/// @brief Z-axis rotation in deg/sec
extern float gyroZ;

/// @brief BMM350 initialisation status
extern bool BMM350OK;

/// @brief X-axis magnetic field in uT
extern float magX;

/// @brief Y-axis magnetic field in uT
extern float magY;

/// @brief Z-axis magnetic field in uT
extern float magZ;

/// @brief BMI270 temperature in degrees C
extern float imuTemp;

#endif