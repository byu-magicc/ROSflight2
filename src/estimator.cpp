/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
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

#include "estimator.h"

#include "rosflight.h"

namespace rosflight_firmware
{
Estimator::Estimator(ROSflight & _rf)
    : RF_(_rf)
{}

void Estimator::reset_state()
{
  state_.attitude.w = 1.0f;
  state_.attitude.x = 0.0f;
  state_.attitude.y = 0.0f;
  state_.attitude.z = 0.0f;

  state_.angular_velocity.x = 0.0f;
  state_.angular_velocity.y = 0.0f;
  state_.angular_velocity.z = 0.0f;

  state_.roll = 0.0f;
  state_.pitch = 0.0f;
  state_.yaw = 0.0f;

  w1_.x = 0.0f;
  w1_.y = 0.0f;
  w1_.z = 0.0f;

  w2_.x = 0.0f;
  w2_.y = 0.0f;
  w2_.z = 0.0f;

  bias_.x = 0.0f;
  bias_.y = 0.0f;
  bias_.z = 0.0f;

  accel_LPF_.x = 0;
  accel_LPF_.y = 0;
  accel_LPF_.z = -9.80665;

  gyro_LPF_.x = 0;
  gyro_LPF_.y = 0;
  gyro_LPF_.z = 0;

  state_.timestamp_us = RF_.board_.clock_micros();

  extatt_update_next_run_ = false;

  // Clear the unhealthy estimator flag
  RF_.state_manager_.clear_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
}

void Estimator::reset_adaptive_bias()
{
  bias_.x = 0;
  bias_.y = 0;
  bias_.z = 0;
}

void Estimator::init()
{
  last_time_ = 0;
  last_acc_update_us_ = 0;
  last_extatt_update_us_ = 0;
  reset_state();
}

void Estimator::param_change_callback(uint16_t param_id) { (void) param_id; }

void Estimator::run_LPF()
{
  float alpha_acc = RF_.params_.get_param_float(PARAM_ACC_ALPHA);
  //const turbomath::Vector & raw_accel = RF_.sensors_.data().accel;
  accel_LPF_.x = (1.0f - alpha_acc) * RF_.sensors_.get_imu()->accel[0] + alpha_acc * accel_LPF_.x;
  accel_LPF_.y = (1.0f - alpha_acc) * RF_.sensors_.get_imu()->accel[1] + alpha_acc * accel_LPF_.y;
  accel_LPF_.z = (1.0f - alpha_acc) * RF_.sensors_.get_imu()->accel[2] + alpha_acc * accel_LPF_.z;

  float alpha_gyro_xy = RF_.params_.get_param_float(PARAM_GYRO_XY_ALPHA);
  float alpha_gyro_z = RF_.params_.get_param_float(PARAM_GYRO_Z_ALPHA);
  //const turbomath::Vector & raw_gyro = RF_.sensors_.data().gyro;
  gyro_LPF_.x =
    (1.0f - alpha_gyro_xy) * RF_.sensors_.get_imu()->gyro[0] + alpha_gyro_xy * gyro_LPF_.x;
  gyro_LPF_.y =
    (1.0f - alpha_gyro_xy) * RF_.sensors_.get_imu()->gyro[1] + alpha_gyro_xy * gyro_LPF_.y;
  gyro_LPF_.z =
    (1.0f - alpha_gyro_z) * RF_.sensors_.get_imu()->gyro[2] + alpha_gyro_z * gyro_LPF_.z;
}

void Estimator::set_external_attitude_update(const turbomath::Quaternion & q)
{
  extatt_update_next_run_ = true;
  q_extatt_ = q;
}

void Estimator::run()
{
  //
  // Timing Setup
  //

  const uint64_t now_us = RF_.sensors_.get_imu()->header.timestamp;

  if (last_time_ == 0) {
    last_time_ = now_us;
    last_acc_update_us_ = now_us;
    last_extatt_update_us_ = now_us;
    return;
  } else if (now_us < last_time_) {
    // this shouldn't happen
    RF_.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
    last_time_ = now_us;
    return;
  }

  RF_.state_manager_.clear_error(StateManager::ERROR_TIME_GOING_BACKWARDS);

  float dt = (now_us - last_time_) * 1e-6f;
  last_time_ = now_us;
  state_.timestamp_us = now_us;

  // Low-pass filter accel and gyro measurements
  run_LPF();

  //
  // Gyro Correction Term (werr)
  //

  float kp = 0.0f;
  float ki = RF_.params_.get_param_float(PARAM_FILTER_KI);

  turbomath::Vector w_err;

  if (can_use_accel()) {
    // Get error estimated by accelerometer measurement
    w_err = accel_correction();
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP_ACC);

    last_acc_update_us_ = now_us;
  }

  if (can_use_extatt()) {
    // Get error estimated by external attitude measurement. Overwrite any
    // correction based on the accelerometer (assumption: extatt is better).
    w_err = extatt_correction();
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP_EXT);

    // the angular rate correction from external attitude updates occur at a
    // different rate than IMU updates, so it needs to be integrated with a
    // different dt. The following scales the correction term by the timestep
    // ratio so that it is integrated correctly.
    const float extAttDt = (now_us - last_extatt_update_us_) * 1e-6f;
    const float scaleDt = (dt > 0) ? (extAttDt / dt) : 0.0f;
    w_err *= scaleDt;

    last_extatt_update_us_ = now_us;
    extatt_update_next_run_ = false;
  }

  // Crank up the gains for the first few seconds for quick convergence
  if (now_us < static_cast<uint64_t>(RF_.params_.get_param_int(PARAM_INIT_TIME)) * 1000) {
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP_ACC) * 10.0f;
    ki = RF_.params_.get_param_float(PARAM_FILTER_KI) * 10.0f;
  }

  //
  // Composite Bias-Free Angular Rate (wfinal)
  //

  // Integrate biases driven by measured angular error
  // eq 47b Mahony Paper, using correction term w_err found above
  bias_ -= ki * w_err * dt;

  // Build the composite omega vector for kinematic propagation
  // This the stuff inside the p function in eq. 47a - Mahony Paper
  turbomath::Vector wbar = smoothed_gyro_measurement();
  turbomath::Vector wfinal = wbar - bias_ + kp * w_err;

  //
  // Propagate Dynamics
  //

  integrate_angular_rate(state_.attitude, wfinal, dt);

  //
  // Post-Processing
  //

  // Extract Euler Angles for controller
  state_.attitude.get_RPY(&state_.roll, &state_.pitch, &state_.yaw);

  // Save off adjust gyro measurements with estimated biases for control
  state_.angular_velocity = gyro_LPF_ - bias_;

  attitude_.header.timestamp = state_.timestamp_us;

  attitude_.q[0] = state_.attitude.w;
  attitude_.q[1] = state_.attitude.x;
  attitude_.q[2] = state_.attitude.y;
  attitude_.q[3] = state_.attitude.z;

  attitude_.rate[0] = state_.angular_velocity.x;
  attitude_.rate[1] = state_.angular_velocity.y;
  attitude_.rate[2] = state_.angular_velocity.z;

  // If it has been more than 0.5 seconds since the accel update ran and we
  // are supposed to be getting them then trigger an unhealthy estimator error.
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_ACC) && now_us > 500000 + last_acc_update_us_
      && !RF_.params_.get_param_int(PARAM_FIXED_WING)) {
    RF_.state_manager_.set_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
  } else {
    RF_.state_manager_.clear_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
  }
}

bool Estimator::can_use_accel() const
{
  // if we are not using accel, just bail
  if (!RF_.params_.get_param_int(PARAM_FILTER_USE_ACC)) { return false; }

  // current magnitude of LPF'd accelerometer
  const float a_sqrd_norm = accel_LPF_.sqrd_norm();

  // Ideally, gyros would never drift and we would never have to use the accelerometer.
  // Since gyros do drift, we can use the accelerometer (in a non-accelerated state) as
  // another estimate of roll/pitch angles and to make gyro biases observable (except r).
  // Since there is noise, we give some margin to what a "non-accelerated state" means.
  // Establish allowed acceleration deviation from 1g (i.e., non-accelerated flight).
  const float margin = RF_.params_.get_param_float(PARAM_FILTER_ACCEL_MARGIN);
  const float lowerbound = (1.0f - margin) * (1.0f - margin) * 9.80665f * 9.80665f;
  const float upperbound = (1.0f + margin) * (1.0f + margin) * 9.80665f * 9.80665f;

  // if the magnitude of the accel measurement is close to 1g, we can use the
  // accelerometer to correct roll and pitch and estimate gyro biases.
  return (lowerbound < a_sqrd_norm && a_sqrd_norm < upperbound);
}

bool Estimator::can_use_extatt() const { return extatt_update_next_run_; }

turbomath::Vector Estimator::accel_correction() const
{
  // turn measurement into a unit vector
  turbomath::Vector a = accel_LPF_.normalized();

  // Get the quaternion from accelerometer (low-frequency measure q)
  // (Not in either paper)
  turbomath::Quaternion q_acc_inv(g_, a);

  // Get the error quaternion between observer and low-freq q
  // Below Eq. 45 Mahony Paper
  turbomath::Quaternion q_tilde = q_acc_inv * state_.attitude;

  // Correction Term of Eq. 47a and 47b Mahony Paper
  // w_acc = 2*s_tilde*v_tilde
  turbomath::Vector w_acc;
  w_acc.x = -2.0f * q_tilde.w * q_tilde.x;
  w_acc.y = -2.0f * q_tilde.w * q_tilde.y;
  w_acc.z = 0.0f; // Don't correct z, because it's unobservable from the accelerometer

  return w_acc;
}

turbomath::Vector Estimator::extatt_correction() const
{
  // DCM rows of attitude estimate and external measurement (world w.r.t body).
  // These are the world axes from the perspective of the body frame.
  // Note: If we extracted cols it would be body w.r.t world.
  turbomath::Vector xhat_BW, yhat_BW, zhat_BW;
  turbomath::Vector xext_BW, yext_BW, zext_BW;

  // extract rows of rotation matrix from quaternion attitude estimate
  quaternion_to_dcm(state_.attitude, xhat_BW, yhat_BW, zhat_BW);

  // extract rows of rotation matrix from quaternion external attitude
  quaternion_to_dcm(q_extatt_, xext_BW, yext_BW, zext_BW);

  // calculate cross products of corresponding axes as an error metric. For example, if vehicle were
  // level but extatt had a different yaw angle than the internal estimate, xext_BW.cross(xhat_BW)
  // would be a measure of how the filter needs to update in order to the internal estimate's yaw
  // to closer to the extatt measurement. This is done for each axis.
  turbomath::Vector w_ext =
    xext_BW.cross(xhat_BW) + yext_BW.cross(yhat_BW) + zext_BW.cross(zhat_BW);

  return w_ext;
}

turbomath::Vector Estimator::smoothed_gyro_measurement()
{
  turbomath::Vector wbar;
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_QUAD_INT)) {
    // Quadratic Interpolation (Eq. 14 Casey Paper)
    // this step adds 12 us on the STM32F10x chips
    wbar = (w2_ / -12.0f) + w1_ * (8.0f / 12.0f) + gyro_LPF_ * (5.0f / 12.0f);
    w2_ = w1_;
    w1_ = gyro_LPF_;
  } else {
    wbar = gyro_LPF_;
  }

  return wbar;
}

void Estimator::integrate_angular_rate(turbomath::Quaternion & quat,
                                       const turbomath::Vector & omega, const float dt) const
{
  // only propagate if we've moved
  // TODO[PCL]: Will this ever be true? We should add a margin to this
  const float sqrd_norm_w = omega.sqrd_norm();
  if (sqrd_norm_w == 0.0f) { return; }

  // for convenience
  const float &p = omega.x, &q = omega.y, &r = omega.z;

  if (RF_.params_.get_param_int(PARAM_FILTER_USE_MAT_EXP)) {
    // Matrix Exponential Approximation (From Attitude Representation and Kinematic
    // Propagation for Low-Cost UAVs by Robert T. Casey)
    // (Eq. 12 Casey Paper)
    // This adds 90 us on STM32F10x chips
    float norm_w = sqrtf(sqrd_norm_w);
    float t1 = cosf((norm_w * dt) / 2.0f);
    float t2 = 1.0f / norm_w * sinf((norm_w * dt) / 2.0f);
    quat.w = t1 * quat.w + t2 * (-p * quat.x - q * quat.y - r * quat.z);
    quat.x = t1 * quat.x + t2 * (p * quat.w + r * quat.y - q * quat.z);
    quat.y = t1 * quat.y + t2 * (q * quat.w - r * quat.x + p * quat.z);
    quat.z = t1 * quat.z + t2 * (r * quat.w + q * quat.x - p * quat.y);
    quat.normalize();
  } else {
    // Euler Integration
    // (Eq. 47a Mahony Paper)
    turbomath::Quaternion qdot(
      0.5f * (-p * quat.x - q * quat.y - r * quat.z), 0.5f * (p * quat.w + r * quat.y - q * quat.z),
      0.5f * (q * quat.w - r * quat.x + p * quat.z), 0.5f * (r * quat.w + q * quat.x - p * quat.y));
    quat.w += qdot.w * dt;
    quat.x += qdot.x * dt;
    quat.y += qdot.y * dt;
    quat.z += qdot.z * dt;
    quat.normalize();
  }
}

void Estimator::quaternion_to_dcm(const turbomath::Quaternion & q, turbomath::Vector & X,
                                  turbomath::Vector & Y, turbomath::Vector & Z) const
{
  // R(q) = [X.x X.y X.z]
  //        [Y.x Y.y Y.z]
  //        [Z.x Z.y Z.z]

  const float &w = q.w, &x = q.x, &y = q.y, &z = q.z;
  X.x = 1.0f - 2.0f * (y * y + z * z);
  X.y = 2.0f * (x * y - z * w);
  X.z = 2.0f * (x * z + y * w);
  Y.x = 2.0f * (x * y + z * w);
  Y.y = 1.0f - 2.0f * (x * x + z * z);
  Y.z = 2.0f * (y * z - x * w);
  Z.x = 2.0f * (x * z - y * w);
  Z.y = 2.0f * (y * z + x * w);
  Z.z = 1.0f - 2.0f * (x * x + y * y);
}

} // namespace rosflight_firmware
