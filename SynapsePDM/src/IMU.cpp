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

#include <Arduino.h>
#include <IMU.h>
#include <Globals.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Wire.h>
#include <bmm350.h>

static BMI270 imu;
static struct bmm350_dev magnetometer;
static uint8_t magnetometerAddress = BMM350_I2C_ADSEL_SET_LOW;
static bool bmm350PresentAtStartup = false;
static bool bmm350StartupDetectionComplete = false;
static uint8_t bmm350ConsecutiveZeroReads = 0;
static uint32_t nextMagnetometerRecoveryAt = 0;

float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float magX;
float magY;
float magZ;
float imuTemp;
bool IMUOK;
bool BMM350OK;

static void ClearImuData()
{
  accelX = 0.0f;
  accelY = 0.0f;
  accelZ = 0.0f;
  gyroX = 0.0f;
  gyroY = 0.0f;
  gyroZ = 0.0f;
  imuTemp = 0.0f;
}

static void ClearMagData()
{
  magX = 0.0f;
  magY = 0.0f;
  magZ = 0.0f;
}

static void ScheduleMagnetometerRecovery(uint32_t delayMs)
{
  nextMagnetometerRecoveryAt = millis() + delayMs;
}

static bool InitialisePrimaryIMU()
{
  IMUOK = false;

  Wire.begin();
  Wire.setClock(I2C_BUS_SPEED);

  int8_t err = BMI2_OK;
  IMUOK = !imu.beginI2C(BMI2_I2C_PRIM_ADDR);

  if (!IMUOK)
  {
    return false;
  }

  err |= imu.enableFeature(BMI2_ACCEL);
  err |= imu.enableFeature(BMI2_GYRO);
  err |= imu.enableFeature(BMI2_ANY_MOTION);

  bmi2_sens_config accelConfig;
  accelConfig.type = BMI2_ACCEL;
  accelConfig.cfg.acc.odr = BMI2_ACC_ODR_50HZ;
  accelConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
  accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  accelConfig.cfg.acc.range = BMI2_ACC_RANGE_2G;
  err = imu.setConfig(accelConfig);

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
  return IMUOK;
}

static bool IsZeroMagReading(const struct bmm350_mag_temp_data &magData)
{
  return magData.x == 0.0f && magData.y == 0.0f && magData.z == 0.0f;
}

static BMM350_INTF_RET_TYPE Bmm350ReadRegister(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if (reg_data == nullptr || intf_ptr == nullptr)
  {
    return BMM350_E_NULL_PTR;
  }

  uint8_t deviceAddress = *static_cast<uint8_t *>(intf_ptr);

  Wire.beginTransmission(deviceAddress);
  Wire.write(reg_addr);
  if (Wire.endTransmission(false) != 0)
  {
    return BMM350_E_COM_FAIL;
  }

  if (Wire.requestFrom(static_cast<int>(deviceAddress), static_cast<int>(len)) != static_cast<int>(len))
  {
    while (Wire.available())
    {
      Wire.read();
    }

    return BMM350_E_COM_FAIL;
  }

  for (uint32_t index = 0; index < len; index++)
  {
    if (!Wire.available())
    {
      return BMM350_E_COM_FAIL;
    }

    reg_data[index] = static_cast<uint8_t>(Wire.read());
  }

  return BMM350_INTF_RET_SUCCESS;
}

static BMM350_INTF_RET_TYPE Bmm350WriteRegister(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  if (reg_data == nullptr || intf_ptr == nullptr)
  {
    return BMM350_E_NULL_PTR;
  }

  uint8_t deviceAddress = *static_cast<uint8_t *>(intf_ptr);

  Wire.beginTransmission(deviceAddress);
  Wire.write(reg_addr);
  for (uint32_t index = 0; index < len; index++)
  {
    Wire.write(reg_data[index]);
  }

  if (Wire.endTransmission() != 0)
  {
    return BMM350_E_COM_FAIL;
  }

  return BMM350_INTF_RET_SUCCESS;
}

static void Bmm350DelayUs(uint32_t period, void *intf_ptr)
{
  (void)intf_ptr;

  if (period >= 1000)
  {
    delay(period / 1000);
    period %= 1000;
  }

  if (period > 0)
  {
    delayMicroseconds(period);
  }
}

static void InitialiseMagnetometer()
{
  ClearMagData();
  bmm350ConsecutiveZeroReads = 0;
  BMM350OK = false;
  magnetometer = {};
  magnetometer.intf_ptr = &magnetometerAddress;
  magnetometer.read = Bmm350ReadRegister;
  magnetometer.write = Bmm350WriteRegister;
  magnetometer.delay_us = Bmm350DelayUs;

  int8_t err = bmm350_init(&magnetometer);
  if (err == BMM350_OK)
  {
    err = bmm350_set_odr_performance(BMM350_DATA_RATE_50HZ, BMM350_AVERAGING_4, &magnetometer);
  }

  if (err == BMM350_OK)
  {
    err = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &magnetometer);
  }

  if (err == BMM350_OK)
  {
    err = bmm350_set_powermode(BMM350_NORMAL_MODE, &magnetometer);
  }

  BMM350OK = (err == BMM350_OK);

  if (BMM350OK)
  {
    nextMagnetometerRecoveryAt = 0;
  }
  else
  {
    ScheduleMagnetometerRecovery(BMM350_RECOVERY_RETRY_MS);
  }

  if (!bmm350StartupDetectionComplete)
  {
    bmm350PresentAtStartup = BMM350OK;
  }
}

static bool AttemptMagnetometerRecovery(bool force)
{
  if (!bmm350PresentAtStartup)
  {
    BMM350OK = false;
    ClearMagData();
    return false;
  }

  if (!force && nextMagnetometerRecoveryAt != 0 && (int32_t)(millis() - nextMagnetometerRecoveryAt) < 0)
  {
    return false;
  }

  InitialiseMagnetometer();
  return BMM350OK;
}

void ReinitialiseMagnetometer()
{
  ClearMagData();
  bmm350ConsecutiveZeroReads = 0;
  BMM350OK = false;
  ScheduleMagnetometerRecovery(BMM350_WAKE_RECOVERY_DELAY_MS);
}

void InitialiseIMU()
{
  ClearImuData();
  nextMagnetometerRecoveryAt = 0;
  InitialiseMagnetometer();
  bmm350StartupDetectionComplete = true;
  InitialisePrimaryIMU();
}

void ReinitialiseIMUAfterWake()
{
  ClearImuData();
  InitialisePrimaryIMU();
  ReinitialiseMagnetometer();
}

void SleepIMU(bool allowMotionWake)
{
  if (!IMUOK)
  {
    return;
  }

  int8_t err = BMI2_OK;

  err |= imu.enableAdvancedPowerSave();

  if (allowMotionWake)
  {
    err |= imu.enableFeature(BMI2_ACCEL);
    err |= imu.enableFeature(BMI2_ANY_MOTION);
    err |= imu.disableFeature(BMI2_GYRO);
    err |= imu.setAccelPowerMode(BMI2_POWER_OPT_MODE);
    err |= imu.setAccelODR(BMI2_ACC_ODR_25HZ);
    err |= imu.setAccelFilterBandwidth(BMI2_ACC_NORMAL_AVG4);
  }
  else
  {
    err |= imu.disableFeature(BMI2_ANY_MOTION);
    err |= imu.disableFeature(BMI2_GYRO);
    err |= imu.disableFeature(BMI2_ACCEL);
  }

  if (err == BMI2_OK)
  {
    gyroX = 0.0f;
    gyroY = 0.0f;
    gyroZ = 0.0f;
  }

  IMUOK = (err == BMI2_OK);
}

void ReadIMU()
{
  if (IMUOK)
  {
    imu.getSensorData();

    accelX = imu.data.accelX;
    accelY = imu.data.accelY;
    accelZ = imu.data.accelZ;
    gyroX = imu.data.gyroX;
    gyroY = imu.data.gyroY;
    gyroZ = imu.data.gyroZ;

    float measuredTemperature = 0.0f;
    if (imu.getTemperature(&measuredTemperature) == BMI2_OK)
    {
      imuTemp = measuredTemperature;
    }
  }
  else
  {
    ClearImuData();
  }

  if (BMM350OK)
  {
    struct bmm350_mag_temp_data magData = {0};
    if (bmm350_get_compensated_mag_xyz_temp_data(&magData, &magnetometer) == BMM350_OK)
    {
      if (IsZeroMagReading(magData))
      {
        bmm350ConsecutiveZeroReads++;
        if (bmm350ConsecutiveZeroReads >= BMM350_ZERO_RECOVERY_THRESHOLD && AttemptMagnetometerRecovery(false))
        {
          if (bmm350_get_compensated_mag_xyz_temp_data(&magData, &magnetometer) == BMM350_OK && !IsZeroMagReading(magData))
          {
            magX = magData.x;
            magY = magData.y;
            magZ = magData.z;
            bmm350ConsecutiveZeroReads = 0;
          }
          else
          {
            ClearMagData();
          }
        }
        else
        {
          ClearMagData();
        }
      }
      else
      {
        magX = magData.x;
        magY = magData.y;
        magZ = magData.z;
        bmm350ConsecutiveZeroReads = 0;
      }
    }
    else
    {
      ClearMagData();
      BMM350OK = false;
      ScheduleMagnetometerRecovery(BMM350_RECOVERY_RETRY_MS);
      AttemptMagnetometerRecovery(false);
    }
  }
  else
  {
    ClearMagData();
    AttemptMagnetometerRecovery(false);
  }
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
