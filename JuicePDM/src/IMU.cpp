/*  IMU.cpp IMU functions.
    Copyright (c) 2025 Joe Mann.  All right reserved.

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
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();

  IMUOK = !imu.beginI2C(BMI2_I2C_PRIM_ADDR);

  // Enable IMU features
  err |= imu.enableFeature(BMI2_ACCEL);
  err |= imu.enableFeature(BMI2_GYRO);
  err |= imu.enableFeature(BMI2_ANY_MOTION);

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

  // .duration  - 5 = 20ms
  // .threshold - 170 = 83mg
  // .select_x  - Enabled
  // .select_y  - Enabled
  // .select_z  - Enabled
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

#ifdef DEBUG
  Serial.print("Set IMU config: ");
  Serial.println(err);
  delay(100);
#endif
}

void DisableMotionDetect()
{
  int8_t err = BMI2_OK;
  imu.disableFeature(BMI2_ANY_MOTION);
  IMUOK = !err;
}
