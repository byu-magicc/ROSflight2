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

#ifndef ROSFLIGHT_FIRMWARE_ESTIMATOR_H
#define ROSFLIGHT_FIRMWARE_ESTIMATOR_H

#include "interface/param_listener.h"

#include <turbomath/turbomath.h>

#include <cmath>
#include <cstdbool>
#include <cstdint>

namespace rosflight_firmware
{
class ROSflight;

class Estimator : public ParamListenerInterface
{

public:
  typedef struct //__attribute__((__packed__))
  {
    uint64_t timestamp; // us, time of data read complete
    float q[4];         // quaternions
    float rate[3];
  } AttitudeStruct;

  struct State
  {
    turbomath::Vector angular_velocity;
    turbomath::Quaternion attitude;
    float roll;
    float pitch;
    float yaw;
    uint64_t timestamp_us;
  };

  Estimator(ROSflight & _rf);

  inline const State & state() const { return state_; }

  inline const turbomath::Vector & bias() { return bias_; }

  inline const turbomath::Vector & accLPF() { return accel_LPF_; }

  inline const turbomath::Vector & gyroLPF() { return gyro_LPF_; }

  void init();
  void param_change_callback(uint16_t param_id) override;
  void run();
  void reset_state();
  void reset_adaptive_bias();
  void set_external_attitude_update(const turbomath::Quaternion & q);

  AttitudeStruct * get_attitude(void) { return &attitude_; }

private:
  AttitudeStruct attitude_;
  const turbomath::Vector g_ = {0.0f, 0.0f, -1.0f};

  ROSflight & RF_;
  State state_;

  uint64_t last_time_;
  uint64_t last_acc_update_us_;
  uint64_t last_extatt_update_us_;

  turbomath::Vector w1_;
  turbomath::Vector w2_;

  turbomath::Vector bias_;

  turbomath::Vector accel_LPF_;
  turbomath::Vector gyro_LPF_;

  turbomath::Vector w_acc_;

  bool extatt_update_next_run_;
  turbomath::Quaternion q_extatt_;

  void run_LPF();

  bool can_use_accel() const;
  bool can_use_extatt() const;
  turbomath::Vector accel_correction() const;
  turbomath::Vector extatt_correction() const;
  turbomath::Vector smoothed_gyro_measurement();
  void integrate_angular_rate(turbomath::Quaternion & quat, const turbomath::Vector & omega,
                              const float dt) const;
  void quaternion_to_dcm(const turbomath::Quaternion & q, turbomath::Vector & X,
                         turbomath::Vector & Y, turbomath::Vector & Z) const;
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_ESTIMATOR_H
