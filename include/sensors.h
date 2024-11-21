/*
 * Copyright (c) 2017, James Jackson, Daniel Koch, and Craig Bidstrup,
 * BYU MAGICC Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSFLIGHT_FIRMWARE_SENSORS_H
#define ROSFLIGHT_FIRMWARE_SENSORS_H

#include "param_listener.h"

#include <turbomath/turbomath.h>

#include <cstdbool>
#include <cstdint>
#include <cstring>

#include <estimator.h>

#include <rosflight_structs.h>

namespace rosflight_firmware
{
class ROSflight;

class Sensors : public ParamListenerInterface
{
public:
  PressureStruct * get_diff_pressure(void) { return &diff_pressure_; }
  PressureStruct * get_baro(void) { return &baro_; }
  RangeStruct * get_sonar(void) { return &sonar_; }
  ImuStruct * get_imu(void) { return &imu_; }
  BatteryStruct * get_battery(void) { return &battery_; }
  MagStruct * get_mag(void) { return &mag_; }
  GnssStruct * get_gnss(void) { return &gnss_; }

  Sensors(ROSflight & rosflight);

  void init();
  got_flags run();
  void param_change_callback(uint16_t param_id) override;

  // Calibration Functions
  bool start_imu_calibration(void);
  bool start_gyro_calibration(void);
  bool start_baro_calibration(void);
  bool start_diff_pressure_calibration(void);
  bool gyro_calibration_complete(void);

private:
  // Data
  PressureStruct diff_pressure_ = {};
  PressureStruct baro_ = {};
  RangeStruct sonar_ = {};
  ImuStruct imu_ = {};
  BatteryStruct battery_ = {};
  MagStruct mag_ = {};
  GnssStruct gnss_ = {};

  void rotate_imu_in_place(ImuStruct * imu, turbomath::Quaternion q);
  turbomath::Quaternion fcu_orientation_ = {1, 0, 0, 0};

  static const int SENSOR_CAL_DELAY_CYCLES;
  static const int SENSOR_CAL_CYCLES;
  static const float BARO_MAX_CALIBRATION_VARIANCE;
  static const float DIFF_PRESSURE_MAX_CALIBRATION_VARIANCE;

  enum : uint8_t
  {
    BAROMETER,
    GNSS,
    DIFF_PRESSURE,
    SONAR,
    MAGNETOMETER,
    BATTERY_MONITOR,
    NUM_LOW_PRIORITY_SENSORS
  };

  ROSflight & rf_;

  float accel_[3] = {0, 0, 0};
  float gyro_[3] = {0, 0, 0};

  bool calibrating_acc_flag_ = false;
  bool calibrating_gyro_flag_ = false;
  void init_imu();
  void calibrate_accel(void);
  void calibrate_gyro(void);
  void calibrate_baro(void);
  void calibrate_diff_pressure(void);
  void correct_imu(void);
  void correct_mag(void);
  void correct_baro(void);
  void correct_diff_pressure(void);
  void update_battery_monitor_multipliers(void);

  // IMU calibration
  uint16_t gyro_calibration_count_ = 0;
  turbomath::Vector gyro_sum_ = {0, 0, 0};
  uint16_t accel_calibration_count_ = 0;
  turbomath::Vector acc_sum_ = {0, 0, 0};
  const turbomath::Vector gravity_ = {0.0f, 0.0f, 9.80665f};
  float acc_temp_sum_ = 0.0f;
  turbomath::Vector max_ = {-1000.0f, -1000.0f, -1000.0f};
  turbomath::Vector min_ = {1000.0f, 1000.0f, 1000.0f};

  // Baro Calibration
  bool baro_calibrated_ = false;
  uint16_t baro_calibration_count_ = 0;
  uint32_t last_baro_cal_iter_ms_ = 0;
  float baro_calibration_mean_ = 0.0f;
  float baro_calibration_var_ = 0.0f;

  // Diff Pressure Calibration
  bool diff_pressure_calibrated_ = false;
  uint16_t diff_pressure_calibration_count_ = 0;
  uint32_t last_diff_pressure_cal_iter_ms_ = 0;
  float diff_pressure_calibration_mean_ = 0.0f;
  float diff_pressure_calibration_var_ = 0.0f;

  // Battery Monitor
  float battery_voltage_alpha_{0.995};
  float battery_current_alpha_{0.995};
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_SENSORS_H
