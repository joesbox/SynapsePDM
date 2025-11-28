/*  IMU.cpp IMU functions.
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

#include <IMU.h>
#include <SparkFun_BMI270_Arduino_Library.h>

BMI270 imu;

float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
bool IMUOK;

void InitialiseIMU()
{
  IMUOK = false;
  int8_t err = BMI2_OK;


  IMUOK = !imu.beginI2C(BMI2_I2C_PRIM_ADDR);

  // Enable IMU features
  err |= imu.enableFeature(BMI2_ACCEL);
  err |= imu.enableFeature(BMI2_GYRO);
  err |= imu.enableFeature(BMI2_ANY_MOTION);

  // Set accelerometer config
  bmi2_sens_config accelConfig;
  accelConfig.type = BMI2_ACCEL;
  accelConfig.cfg.acc.odr = BMI2_ACC_ODR_50HZ;
  accelConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
  accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  accelConfig.cfg.acc.range = BMI2_ACC_RANGE_2G;
  err = imu.setConfig(accelConfig);

  // Set gyroscope config
  bmi2_sens_config gyroConfig;
  gyroConfig.type = BMI2_GYRO;
  gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
  gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
  gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  gyroConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_250;
  gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_125;
  gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
  err = imu.setConfig(gyroConfig);

  IMUOK = !err;
}

void ReadIMU()
{
  imu.getSensorData();

  accelX = imu.data.accelX;
  accelY = imu.data.accelY;
  accelZ = imu.data.accelZ;
  gyroX = imu.data.gyroX;
  gyroY = imu.data.gyroY;
  gyroZ = imu.data.gyroZ;
}

void EnableMotionDetect()
{
  int8_t err = BMI2_OK;

  bmi2_sens_config anyMotionConfig;
  anyMotionConfig.type = BMI2_ANY_MOTION;
  anyMotionConfig.cfg.any_motion.duration = 1;
  anyMotionConfig.cfg.any_motion.threshold = 170;
  anyMotionConfig.cfg.any_motion.select_x = BMI2_ENABLE;
  anyMotionConfig.cfg.any_motion.select_y = BMI2_ENABLE;
  anyMotionConfig.cfg.any_motion.select_z = BMI2_ENABLE;
  err |= imu.setConfig(anyMotionConfig);

  bmi2_int_pin_config intPinConfig;
  intPinConfig.pin_type = BMI2_INT1;
  intPinConfig.int_latch = BMI2_INT_NON_LATCH;
  intPinConfig.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
  intPinConfig.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  intPinConfig.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
  err |= imu.setInterruptPinConfig(intPinConfig);
  err |= imu.mapInterruptToPin(BMI2_ANY_MOTION_INT, BMI2_INT1);
  IMUOK = !err;
}

void DisableMotionDetect()
{
  int8_t err = BMI2_OK;
  imu.disableFeature(BMI2_ANY_MOTION);
  IMUOK = !err;
}
