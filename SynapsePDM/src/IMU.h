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

/// @brief Initialise the IMU
void InitialiseIMU();

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

#endif