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

#include "sensors.h"

#include "param.h"

#include "rosflight.h"

#include <turbomath/turbomath.h>

#include <cmath>
#include <cstdbool>
#include <cstdint>

namespace rosflight_firmware
{
const int Sensors::SENSOR_CAL_DELAY_CYCLES = 128;
const int Sensors::SENSOR_CAL_CYCLES = 127;

const float Sensors::BARO_MAX_CALIBRATION_VARIANCE = 25.0;           // standard dev about 0.2 m
const float Sensors::DIFF_PRESSURE_MAX_CALIBRATION_VARIANCE = 100.0; // standard dev about 3 m/s

Sensors::Sensors(ROSflight & rosflight)
    : rf_(rosflight)
{}

void Sensors::init()
{
  // clear the IMU read error
  rf_.state_manager_.clear_error(StateManager::ERROR_IMU_NOT_RESPONDING);
  rf_.board_.sensors_init();

  init_imu();

  this->update_battery_monitor_multipliers();
}

void Sensors::init_imu()
{
  // Quaternion to compensate for FCU orientation
  float roll = rf_.params_.get_param_float(PARAM_FC_ROLL) * 0.017453293;
  float pitch = rf_.params_.get_param_float(PARAM_FC_PITCH) * 0.017453293;
  float yaw = rf_.params_.get_param_float(PARAM_FC_YAW) * 0.017453293;
  fcu_orientation_ = turbomath::Quaternion(roll, pitch, yaw);

  // See if the IMU is uncalibrated, and throw an error if it is
  if (rf_.params_.get_param_float(PARAM_ACC_X_BIAS) == 0.0
      && rf_.params_.get_param_float(PARAM_ACC_Y_BIAS) == 0.0
      && rf_.params_.get_param_float(PARAM_ACC_Z_BIAS) == 0.0
      && rf_.params_.get_param_float(PARAM_GYRO_X_BIAS) == 0.0
      && rf_.params_.get_param_float(PARAM_GYRO_Y_BIAS) == 0.0
      && rf_.params_.get_param_float(PARAM_GYRO_Z_BIAS) == 0.0) {
    rf_.state_manager_.set_error(StateManager::ERROR_UNCALIBRATED_IMU);
  }
}

void Sensors::param_change_callback(uint16_t param_id)
{
  switch (param_id) {
    case PARAM_FC_ROLL:
    case PARAM_FC_PITCH:
    case PARAM_FC_YAW:
      init_imu();
      break;
    case PARAM_BATTERY_VOLTAGE_MULTIPLIER:
    case PARAM_BATTERY_CURRENT_MULTIPLIER:
      update_battery_monitor_multipliers();
      break;
    case PARAM_BATTERY_VOLTAGE_ALPHA:
      battery_voltage_alpha_ = rf_.params_.get_param_float(PARAM_BATTERY_VOLTAGE_ALPHA);
      break;
    case PARAM_BATTERY_CURRENT_ALPHA:
      battery_current_alpha_ = rf_.params_.get_param_float(PARAM_BATTERY_CURRENT_ALPHA);
      break;
    default:
      // do nothing
      break;
  }
}

// This could be written out instead of using turbomath.
void Sensors::rotate_imu_in_place(ImuStruct * imu, turbomath::Quaternion rotation)
{
  turbomath::Vector accel;
  accel.x = imu->accel[0];
  accel.y = imu->accel[1];
  accel.z = imu->accel[2];

  accel = rotation * accel;

  imu->accel[0] = accel.x;
  imu->accel[1] = accel.y;
  imu->accel[2] = accel.z;

  turbomath::Vector gyro;
  gyro.x = imu->gyro[0];
  gyro.y = imu->gyro[1];
  gyro.z = imu->gyro[2];

  gyro = rotation * gyro;

  imu->gyro[0] = gyro.x;
  imu->gyro[1] = gyro.y;
  imu->gyro[2] = gyro.z;
}

got_flags Sensors::run()
{
  got_flags got = {};

  // IMU:
  if ((got.imu = rf_.board_.imu_read(&imu_))) {
    rf_.state_manager_.clear_error(StateManager::ERROR_IMU_NOT_RESPONDING);
    rotate_imu_in_place(&imu_, fcu_orientation_);

    if (calibrating_acc_flag_) { calibrate_accel(); }
    if (calibrating_gyro_flag_) { calibrate_gyro(); }

    correct_imu();
  }

  // GNSS:
  got.gnss = rf_.board_.gnss_read(&gnss_);

  // BAROMETER:
  if ((got.baro = rf_.board_.baro_read(&baro_))) { correct_baro(); }

  // MAGNETOMETER:
  if ((got.mag = rf_.board_.mag_read(&mag_))) { correct_mag(); }

  // DIFF_PRESSURE:
  if ((got.diff_pressure = rf_.board_.diff_pressure_read(&diff_pressure_))) {
    correct_diff_pressure();
  }

  // SONAR:
  got.sonar = rf_.board_.sonar_read(&sonar_);

  // BATTERY_MONITOR:
  got.battery = rf_.board_.battery_read(&battery_);

  // RC
  got.rc = rf_.board_.rc_read(&rc_);

  return got;
}

bool Sensors::start_imu_calibration(void)
{
  start_gyro_calibration();

  calibrating_acc_flag_ = true;
  rf_.params_.set_param_float(PARAM_ACC_X_BIAS, 0.0);
  rf_.params_.set_param_float(PARAM_ACC_Y_BIAS, 0.0);
  rf_.params_.set_param_float(PARAM_ACC_Z_BIAS, 0.0);
  return true;
}

bool Sensors::start_gyro_calibration(void)
{
  calibrating_gyro_flag_ = true;
  rf_.params_.set_param_float(PARAM_GYRO_X_BIAS, 0.0);
  rf_.params_.set_param_float(PARAM_GYRO_Y_BIAS, 0.0);
  rf_.params_.set_param_float(PARAM_GYRO_Z_BIAS, 0.0);
  return true;
}

bool Sensors::start_baro_calibration()
{
  baro_calibration_mean_ = 0.0f;
  baro_calibration_var_ = 0.0f;
  baro_calibration_count_ = 0;
  baro_calibrated_ = false;
  rf_.params_.set_param_float(PARAM_BARO_BIAS, 0.0f);
  return true;
}

bool Sensors::start_diff_pressure_calibration()
{
  diff_pressure_calibration_mean_ = 0.0f;
  diff_pressure_calibration_var_ = 0.0f;
  diff_pressure_calibration_count_ = 0;
  diff_pressure_calibrated_ = false;
  rf_.params_.set_param_float(PARAM_DIFF_PRESS_BIAS, 0.0f);
  return true;
}

bool Sensors::gyro_calibration_complete(void) { return !calibrating_gyro_flag_; }

//======================================================================
// Calibration Functions

void Sensors::calibrate_gyro()
{
  gyro_sum_.x += imu_.gyro[0];
  gyro_sum_.y += imu_.gyro[1];
  gyro_sum_.z += imu_.gyro[2];

  gyro_calibration_count_++;

  if (gyro_calibration_count_ > 1000) {
    // Gyros are simple.  Just find the average during the calibration
    turbomath::Vector gyro_bias = gyro_sum_ / static_cast<float>(gyro_calibration_count_);

    if (gyro_bias.norm() < 1.0) {
      rf_.params_.set_param_float(PARAM_GYRO_X_BIAS, gyro_bias.x);
      rf_.params_.set_param_float(PARAM_GYRO_Y_BIAS, gyro_bias.y);
      rf_.params_.set_param_float(PARAM_GYRO_Z_BIAS, gyro_bias.z);

      // Tell the estimator to reset it's bias estimate, because it should be zero now
      rf_.estimator_.reset_adaptive_bias();

      // Tell the state manager that we just completed a gyro calibration
      rf_.state_manager_.set_event(StateManager::EVENT_CALIBRATION_COMPLETE);
    } else {
      // Tell the state manager that we just failed a gyro calibration
      rf_.state_manager_.set_event(StateManager::EVENT_CALIBRATION_FAILED);
      rf_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR,
                            "Too much movement for gyro cal");
    }

    // reset calibration in case we do it again
    calibrating_gyro_flag_ = false;
    gyro_calibration_count_ = 0;
    gyro_sum_.x = 0.0f;
    gyro_sum_.y = 0.0f;
    gyro_sum_.z = 0.0f;
  }
}

turbomath::Vector vector_max(turbomath::Vector a, turbomath::Vector b)
{
  return turbomath::Vector(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z);
}

turbomath::Vector vector_min(turbomath::Vector a, turbomath::Vector b)
{
  return turbomath::Vector(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z);
}

void Sensors::calibrate_accel(void)
{
  //acc_sum_ = acc_sum_ + data_.accel + gravity_;
  acc_sum_.x += imu_.accel[0] + gravity_.x;
  acc_sum_.y += imu_.accel[1] + gravity_.y;
  acc_sum_.z += imu_.accel[2] + gravity_.z;

  acc_temp_sum_ += imu_.temperature;

  //  max_ = vector_max(max_, data_.accel);
  //  min_ = vector_min(min_, data_.accel);

  max_.x = max_.x > imu_.accel[0] ? max_.x : imu_.accel[0];
  max_.y = max_.y > imu_.accel[1] ? max_.y : imu_.accel[1];
  max_.z = max_.z > imu_.accel[2] ? max_.z : imu_.accel[2];

  min_.x = min_.x < imu_.accel[0] ? min_.x : imu_.accel[0];
  min_.y = min_.y < imu_.accel[1] ? min_.y : imu_.accel[1];
  min_.z = min_.z < imu_.accel[2] ? min_.z : imu_.accel[2];

  accel_calibration_count_++;

  if (accel_calibration_count_ > 1000) {
    // The temperature bias is calculated using a least-squares regression.
    // This is computationally intensive, so it is done by the companion
    // computer in fcu_io and shipped over to the flight controller.
    turbomath::Vector accel_temp_bias = {rf_.params_.get_param_float(PARAM_ACC_X_TEMP_COMP),
                                         rf_.params_.get_param_float(PARAM_ACC_Y_TEMP_COMP),
                                         rf_.params_.get_param_float(PARAM_ACC_Z_TEMP_COMP)};

    // Figure out the proper accel bias.
    // We have to consider the contribution of temperature during the calibration,
    // Which is why this line is so confusing. What we are doing, is first removing
    // the contribution of temperature to the measurements during the calibration,
    // Then we are dividing by the number of measurements.
    turbomath::Vector accel_bias =
      (acc_sum_ - (accel_temp_bias * acc_temp_sum_)) / static_cast<float>(accel_calibration_count_);

    // Sanity Check -
    // If the accelerometer is upside down or being spun around during the calibration,
    // then don't do anything
    if ((max_ - min_).norm() > 1.0) {
      rf_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR,
                            "Too much movement for IMU cal");
      calibrating_acc_flag_ = false;
    } else {
      // reset the estimated state
      rf_.estimator_.reset_state();
      calibrating_acc_flag_ = false;

      if (accel_bias.norm() < 3.0) {
        rf_.params_.set_param_float(PARAM_ACC_X_BIAS, accel_bias.x);
        rf_.params_.set_param_float(PARAM_ACC_Y_BIAS, accel_bias.y);
        rf_.params_.set_param_float(PARAM_ACC_Z_BIAS, accel_bias.z);
        rf_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO, "IMU offsets captured");

        // clear uncalibrated IMU flag
        rf_.state_manager_.clear_error(StateManager::ERROR_UNCALIBRATED_IMU);
      } else {
        // This usually means the user has the FCU in the wrong orientation, or something is wrong
        // with the board IMU (like it's a cheap chinese clone)
        rf_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR,
                              "large accel bias: norm = %d.%d",
                              static_cast<uint32_t>(accel_bias.norm()),
                              static_cast<uint32_t>(accel_bias.norm() * 1000) % 1000);
      }
    }

    // reset calibration counters in case we do it again
    accel_calibration_count_ = 0;
    acc_sum_.x = 0.0f;
    acc_sum_.y = 0.0f;
    acc_sum_.z = 0.0f;
    acc_temp_sum_ = 0.0f;
    max_.x = -1000.0f;
    max_.y = -1000.0f;
    max_.z = -1000.0f;
    min_.x = 1000.0f;
    min_.y = 1000.0f;
    min_.z = 1000.0f;
  }
}

void Sensors::calibrate_baro()
{
  if (rf_.board_.clock_millis() > last_baro_cal_iter_ms_ + 20) {
    baro_calibration_count_++;

    // calibrate pressure reading to where it should be
    if (baro_calibration_count_ > SENSOR_CAL_DELAY_CYCLES + SENSOR_CAL_CYCLES) {
      // if sample variance within acceptable range, flag calibration as done
      // else reset cal variables and start over
      if (baro_calibration_var_ < BARO_MAX_CALIBRATION_VARIANCE) {
        rf_.params_.set_param_float(PARAM_BARO_BIAS, baro_calibration_mean_);
        // set ground altitude to be pressure altitude at PARAM_BARO_BIAS
        rf_.params_.set_param_float(PARAM_GROUND_LEVEL,
                                    turbomath::alt(rf_.params_.get_param_float(PARAM_BARO_BIAS)));
        baro_calibrated_ = true;
        rf_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,
                              "Baro ground pressure cal successful!");
      } else {
        rf_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR,
                              "Too much movement for barometer ground pressure cal");
      }
      baro_calibration_mean_ = 0.0f;
      baro_calibration_var_ = 0.0f;
      baro_calibration_count_ = 0;
    } else if (baro_calibration_count_ > SENSOR_CAL_DELAY_CYCLES) {
      float measurement = baro_.pressure;
      float delta = measurement - baro_calibration_mean_;
      baro_calibration_mean_ += delta / (baro_calibration_count_ - SENSOR_CAL_DELAY_CYCLES);
      float delta2 = measurement - baro_calibration_mean_;
      baro_calibration_var_ += delta * delta2 / (SENSOR_CAL_CYCLES - 1);
    }
    last_baro_cal_iter_ms_ = rf_.board_.clock_millis();
  }
}

void Sensors::calibrate_diff_pressure()
{
  if (rf_.board_.clock_millis() > last_diff_pressure_cal_iter_ms_ + 20) {
    diff_pressure_calibration_count_++;

    if (diff_pressure_calibration_count_ > SENSOR_CAL_DELAY_CYCLES + SENSOR_CAL_CYCLES) {
      // if sample variance within acceptable range, flag calibration as done
      // else reset cal variables and start over
      if (diff_pressure_calibration_var_ < DIFF_PRESSURE_MAX_CALIBRATION_VARIANCE) {
        rf_.params_.set_param_float(PARAM_DIFF_PRESS_BIAS, diff_pressure_calibration_mean_);
        diff_pressure_calibrated_ = true;
        rf_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO, "Airspeed Cal Successful!");
      } else {
        rf_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR,
                              "Too much movement for diff pressure cal");
      }
      diff_pressure_calibration_mean_ = 0.0f;
      diff_pressure_calibration_var_ = 0.0f;
      diff_pressure_calibration_count_ = 0;
    } else if (diff_pressure_calibration_count_ > SENSOR_CAL_DELAY_CYCLES) {
      float delta = diff_pressure_.pressure - diff_pressure_calibration_mean_;
      diff_pressure_calibration_mean_ +=
        delta / (diff_pressure_calibration_count_ - SENSOR_CAL_DELAY_CYCLES);
      float delta2 = diff_pressure_.pressure - diff_pressure_calibration_mean_;
      diff_pressure_calibration_var_ += delta * delta2 / (SENSOR_CAL_CYCLES - 1);
    }
    last_diff_pressure_cal_iter_ms_ = rf_.board_.clock_millis();
  }
}

//======================================================
// Correction Functions (These apply calibration constants)
void Sensors::correct_imu(void)
{
  // correct according to known biases and temperature compensation
  imu_.accel[0] -= rf_.params_.get_param_float(PARAM_ACC_X_TEMP_COMP) * imu_.temperature
    + rf_.params_.get_param_float(PARAM_ACC_X_BIAS);
  imu_.accel[1] -= rf_.params_.get_param_float(PARAM_ACC_Y_TEMP_COMP) * imu_.temperature
    + rf_.params_.get_param_float(PARAM_ACC_Y_BIAS);
  imu_.accel[2] -= rf_.params_.get_param_float(PARAM_ACC_Z_TEMP_COMP) * imu_.temperature
    + rf_.params_.get_param_float(PARAM_ACC_Z_BIAS);

  imu_.gyro[0] -= rf_.params_.get_param_float(PARAM_GYRO_X_BIAS);
  imu_.gyro[1] -= rf_.params_.get_param_float(PARAM_GYRO_Y_BIAS);
  imu_.gyro[2] -= rf_.params_.get_param_float(PARAM_GYRO_Z_BIAS);
}

void Sensors::correct_mag(void)
{
  // correct according to known hard iron bias
  float mag_hard_x = mag_.flux[0] - rf_.params_.get_param_float(PARAM_MAG_X_BIAS);
  float mag_hard_y = mag_.flux[1] - rf_.params_.get_param_float(PARAM_MAG_Y_BIAS);
  float mag_hard_z = mag_.flux[2] - rf_.params_.get_param_float(PARAM_MAG_Z_BIAS);

  // correct according to known soft iron bias - converts to nT
  mag_.flux[0] = rf_.params_.get_param_float(PARAM_MAG_A11_COMP) * mag_hard_x
    + rf_.params_.get_param_float(PARAM_MAG_A12_COMP) * mag_hard_y
    + rf_.params_.get_param_float(PARAM_MAG_A13_COMP) * mag_hard_z;
  mag_.flux[1] = rf_.params_.get_param_float(PARAM_MAG_A21_COMP) * mag_hard_x
    + rf_.params_.get_param_float(PARAM_MAG_A22_COMP) * mag_hard_y
    + rf_.params_.get_param_float(PARAM_MAG_A23_COMP) * mag_hard_z;
  mag_.flux[2] = rf_.params_.get_param_float(PARAM_MAG_A31_COMP) * mag_hard_x
    + rf_.params_.get_param_float(PARAM_MAG_A32_COMP) * mag_hard_y
    + rf_.params_.get_param_float(PARAM_MAG_A33_COMP) * mag_hard_z;
}

void Sensors::correct_baro(void)
{
  if (!baro_calibrated_) { calibrate_baro(); }
  baro_.altitude = turbomath::alt(baro_.pressure);
}

void Sensors::correct_diff_pressure()
{
  if (!diff_pressure_calibrated_) { calibrate_diff_pressure(); }
  diff_pressure_.pressure -= rf_.params_.get_param_float(PARAM_DIFF_PRESS_BIAS);
  diff_pressure_.speed = turbomath::fsign(diff_pressure_.pressure)
    * sqrt((fabs(diff_pressure_.pressure) / (0.5 * 1.225)));
}

void Sensors::update_battery_monitor_multipliers()
{
  float voltage_multiplier = this->rf_.params_.get_param_float(PARAM_BATTERY_VOLTAGE_MULTIPLIER);
  float current_multiplier = this->rf_.params_.get_param_float(PARAM_BATTERY_CURRENT_MULTIPLIER);
  this->rf_.board_.battery_voltage_set_multiplier(voltage_multiplier);
  this->rf_.board_.battery_current_set_multiplier(current_multiplier);
}

} // namespace rosflight_firmware
