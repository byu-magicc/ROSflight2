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

#include "mixer.h"

#include "rosflight.h"

#include <cstdint>
#include <iostream>

namespace rosflight_firmware
{
Mixer::Mixer(ROSflight & _rf)
    : RF_(_rf)
{
  mixer_to_use_ = nullptr;
}

void Mixer::init() { init_mixing(); }

void Mixer::param_change_callback(uint16_t param_id)
{
  switch (param_id) {
    case PARAM_MIXER:
    case PARAM_MOTOR_RESISTANCE:
    case PARAM_AIR_DENSITY:
    case PARAM_MOTOR_KV:
    case PARAM_NO_LOAD_CURRENT:
    case PARAM_PROP_DIAMETER:
    case PARAM_PROP_CT:
    case PARAM_PROP_CQ:
    case PARAM_NUM_MOTORS:
    case PARAM_VOLT_MAX:
      init_mixing();
      break;
    case PARAM_MOTOR_PWM_SEND_RATE:
    case PARAM_RC_TYPE:
      init_PWM();
      break;
    default:
      // do nothing
      break;
  }
}

void Mixer::init_mixing()
{
  // clear the invalid mixer error
  RF_.state_manager_.clear_error(StateManager::ERROR_INVALID_MIXER);

  uint8_t mixer_choice = RF_.params_.get_param_int(PARAM_MIXER);

  if (mixer_choice >= NUM_MIXERS) {
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR, "Invalid Mixer Choice");

    // set the invalid mixer flag
    RF_.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
    mixer_to_use_ = nullptr;
  } else {
    mixer_to_use_ = array_of_mixers_[mixer_choice];

    if (mixer_to_use_ == &custom_mixing) {
      // If specified, compute the custom mixing matrix per the motor and propeller parameters
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,
                            "Calculating custom mixer values...");
      load_primary_mixer_values();
      
      // TODO: Add these as parameters in the firmware, otherwise we can't control S vs M for custom mixers
      // Add the header values (PWM rate and output type) to mixer
      add_header_to_mixer(&primary_mixer_);
      mixer_to_use_ = &primary_mixer_;
    } else if (mixer_to_use_ != &fixedwing_mixing ||
               mixer_to_use_ != &fixedwing_inverted_vtail_mixing) {
      // Don't invert the fixedwing mixers

      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,
                            "Inverting selected mixing matrix...");

      // Otherwise, invert the selected "canned" matrix
      primary_mixer_ = invert_mixer(mixer_to_use_);

      mixer_to_use_ = &primary_mixer_;
    }

    std::cout << "Mixer: " << std::endl;
    for (auto i : mixer_to_use_->output_type) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
    for (auto i : mixer_to_use_->default_pwm_rate) {
      std::cout << i << " ";
    }
    for (auto i : mixer_to_use_->Fx) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
    for (auto i : mixer_to_use_->Fy) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
    for (auto i : mixer_to_use_->Fz) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
    for (auto i : mixer_to_use_->Qx) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
    for (auto i : mixer_to_use_->Qy) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
    for (auto i : mixer_to_use_->Qz) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
  }

  init_PWM();

  for (int8_t i = 0; i < NUM_TOTAL_OUTPUTS; i++) {
    raw_outputs_[i] = 0.0f;
    outputs_[i] = 0.0f;
  }
}

void Mixer::update_parameters()
{
  R_ = RF_.params_.get_param_float(PARAM_MOTOR_RESISTANCE);
  rho_ = RF_.params_.get_param_float(PARAM_AIR_DENSITY);
  K_V_ = RF_.params_.get_param_float(PARAM_MOTOR_KV);
  K_Q_ = K_V_;
  i_0_ = RF_.params_.get_param_float(PARAM_NO_LOAD_CURRENT);
  D_ = RF_.params_.get_param_float(PARAM_PROP_DIAMETER);
  C_T_ = RF_.params_.get_param_float(PARAM_PROP_CT);
  C_Q_ = RF_.params_.get_param_float(PARAM_PROP_CQ);
  num_motors_ = RF_.params_.get_param_int(PARAM_NUM_MOTORS);
  V_max_ = RF_.params_.get_param_float(PARAM_VOLT_MAX);
}

Mixer::mixer_t Mixer::invert_mixer(const mixer_t* mixer_to_invert)
{
  Eigen::Matrix<float, 6, NUM_MIXER_OUTPUTS> mixer_matrix;
  mixer_matrix.setZero();

  // Convert the mixer_t to an Eigen matrix
  for (int i=0; i<NUM_MIXER_OUTPUTS; i++) {
    mixer_matrix(0, i) = mixer_to_invert->Fx[i];
    mixer_matrix(1, i) = mixer_to_invert->Fy[i];
    mixer_matrix(2, i) = mixer_to_invert->Fz[i];
    mixer_matrix(3, i) = mixer_to_invert->Qx[i];
    mixer_matrix(4, i) = mixer_to_invert->Qy[i];
    mixer_matrix(5, i) = mixer_to_invert->Qz[i];
  }

  // Calculate the pseudoinverse of the mixing matrix using the SVD
  Eigen::JacobiSVD<Eigen::Matrix<float, 6, NUM_MIXER_OUTPUTS>> svd(
    mixer_matrix,
    Eigen::FullPivHouseholderQRPreconditioner | Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<float, NUM_MIXER_OUTPUTS, 6> Sig;
  Sig.setZero();

  // Avoid dividing by zero in the Sigma matrix
  if (svd.singularValues()[0] != 0.0) { Sig(0, 0) = 1.0 / svd.singularValues()[0]; }
  if (svd.singularValues()[1] != 0.0) { Sig(1, 1) = 1.0 / svd.singularValues()[1]; }
  if (svd.singularValues()[2] != 0.0) { Sig(2, 2) = 1.0 / svd.singularValues()[2]; }
  if (svd.singularValues()[3] != 0.0) { Sig(3, 3) = 1.0 / svd.singularValues()[3]; }
  if (svd.singularValues()[4] != 0.0) { Sig(4, 4) = 1.0 / svd.singularValues()[4]; }
  if (svd.singularValues()[5] != 0.0) { Sig(5, 5) = 1.0 / svd.singularValues()[5]; }

  // Pseudoinverse of the mixing matrix
  Eigen::Matrix<float, NUM_MIXER_OUTPUTS, 6> mixer_matrix_pinv =
    svd.matrixV() * Sig * svd.matrixU().transpose();

  // Fill in the mixing matrix from the inverted matrix above
  mixer_t inverted_mixer;

  for (int i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    inverted_mixer.default_pwm_rate[i] = mixer_to_invert->default_pwm_rate[i];

    if (i < RF_.params_.get_param_int(PARAM_NUM_MOTORS)) {
      inverted_mixer.output_type[i] = M;
      inverted_mixer.Fx[i] = mixer_matrix_pinv(i, 0);
      inverted_mixer.Fy[i] = mixer_matrix_pinv(i, 1);
      inverted_mixer.Fz[i] = mixer_matrix_pinv(i, 2);
      inverted_mixer.Qx[i] = mixer_matrix_pinv(i, 3);
      inverted_mixer.Qy[i] = mixer_matrix_pinv(i, 4);
      inverted_mixer.Qz[i] = mixer_matrix_pinv(i, 5);
    } else {
      // Set the rest of the ouput types to NONE
      inverted_mixer.output_type[i] = NONE;
      inverted_mixer.Fx[i] = 0.0;
      inverted_mixer.Fy[i] = 0.0;
      inverted_mixer.Fz[i] = 0.0;
      inverted_mixer.Qx[i] = 0.0;
      inverted_mixer.Qy[i] = 0.0;
      inverted_mixer.Qz[i] = 0.0;
    }
  }

  return inverted_mixer;
}

void Mixer::add_header_to_mixer(mixer_t* mixer)
{
  // Fill in the default PWM rates to use in the header of the mixer
  for (int i = 0; i < NUM_MIXER_OUTPUTS; i++){
    if (i < RF_.params_.get_param_int(PARAM_NUM_MOTORS) || 
        (RF_.params_.get_param_int(PARAM_NUM_MOTORS) > 4 && i < 8)) {
      // This makes sure the PWM groups are correct for the hardware
      mixer->default_pwm_rate[i] = 490;
    } else {
      mixer->default_pwm_rate[i] = 50;
    }
  }
}

void Mixer::load_primary_mixer_values()
{
  // Load the mixer values from the firmware parameters
  primary_mixer_.Fx[0] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_1);
  primary_mixer_.Fy[0] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_1);
  primary_mixer_.Fz[0] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_1);
  primary_mixer_.Qx[0] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_1);
  primary_mixer_.Qy[0] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_1);
  primary_mixer_.Qz[0] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_1);

  primary_mixer_.Fx[1] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_2);
  primary_mixer_.Fy[1] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_2);
  primary_mixer_.Fz[1] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_2);
  primary_mixer_.Qx[1] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_2);
  primary_mixer_.Qy[1] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_2);
  primary_mixer_.Qz[1] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_2);

  primary_mixer_.Fx[2] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_3);
  primary_mixer_.Fy[2] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_3);
  primary_mixer_.Fz[2] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_3);
  primary_mixer_.Qx[2] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_3);
  primary_mixer_.Qy[2] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_3);
  primary_mixer_.Qz[2] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_3);

  primary_mixer_.Fx[3] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_4);
  primary_mixer_.Fy[3] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_4);
  primary_mixer_.Fz[3] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_4);
  primary_mixer_.Qx[3] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_4);
  primary_mixer_.Qy[3] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_4);
  primary_mixer_.Qz[3] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_4);

  primary_mixer_.Fx[4] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_5);
  primary_mixer_.Fy[4] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_5);
  primary_mixer_.Fz[4] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_5);
  primary_mixer_.Qx[4] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_5);
  primary_mixer_.Qy[4] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_5);
  primary_mixer_.Qz[4] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_5);

  primary_mixer_.Fx[5] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_6);
  primary_mixer_.Fy[5] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_6);
  primary_mixer_.Fz[5] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_6);
  primary_mixer_.Qx[5] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_6);
  primary_mixer_.Qy[5] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_6);
  primary_mixer_.Qz[5] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_6);

  primary_mixer_.Fx[6] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_7);
  primary_mixer_.Fy[6] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_7);
  primary_mixer_.Fz[6] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_7);
  primary_mixer_.Qx[6] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_7);
  primary_mixer_.Qy[6] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_7);
  primary_mixer_.Qz[6] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_7);

  primary_mixer_.Fx[7] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_8);
  primary_mixer_.Fy[7] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_8);
  primary_mixer_.Fz[7] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_8);
  primary_mixer_.Qx[7] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_8);
  primary_mixer_.Qy[7] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_8);
  primary_mixer_.Qz[7] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_8);

  primary_mixer_.Fx[8] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_9);
  primary_mixer_.Fy[8] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_9);
  primary_mixer_.Fz[8] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_9);
  primary_mixer_.Qx[8] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_9);
  primary_mixer_.Qy[8] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_9);
  primary_mixer_.Qz[8] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_9);

  primary_mixer_.Fx[9] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_1_10);
  primary_mixer_.Fy[9] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_2_10);
  primary_mixer_.Fz[9] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_3_10);
  primary_mixer_.Qx[9] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_4_10);
  primary_mixer_.Qy[9] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_5_10);
  primary_mixer_.Qz[9] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_6_10);
}

void Mixer::init_PWM()
{
  if (mixer_to_use_ != nullptr) {
    RF_.board_.pwm_init_multi(mixer_to_use_->default_pwm_rate, NUM_MIXER_OUTPUTS);
  } else {
    RF_.board_.pwm_init_multi(esc_calibration_mixing.default_pwm_rate, NUM_MIXER_OUTPUTS);
  }
}

void Mixer::write_motor(uint8_t index, float value)
{
  if (RF_.state_manager_.state().armed) {
    if (value > 1.0) {
      value = 1.0;
    } else if (value < RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE)
               && RF_.params_.get_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED)) {
      value = RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE);
    } else if (value < 0.0) {
      value = 0.0;
    }
  } else {
    value = 0.0;
  }
  raw_outputs_[index] = value;
  RF_.board_.pwm_write(index, raw_outputs_[index]);
}

void Mixer::write_servo(uint8_t index, float value)
{
  if (value > 1.0) {
    value = 1.0;
  } else if (value < -1.0) {
    value = -1.0;
  }
  raw_outputs_[index] = value;
  RF_.board_.pwm_write(index, raw_outputs_[index] * 0.5 + 0.5);
}

void Mixer::set_new_aux_command(aux_command_t new_aux_command)
{
  for (uint8_t i = 0; i < NUM_TOTAL_OUTPUTS; i++) {
    aux_command_.channel[i].type = new_aux_command.channel[i].type;
    aux_command_.channel[i].value = new_aux_command.channel[i].value;
  }
}

float Mixer::mix_multirotor_without_motor_parameters()
{
  Controller::Output commands = RF_.controller_.output();

//  std::cout << "Commands: ";
//  std::cout << commands.Fx << " ";
//  std::cout << commands.Fy << " Fz: ";
//  std::cout << commands.Fz << " ";
//  std::cout << commands.Qx << " ";
//  std::cout << commands.Qy << " ";
//  std::cout << commands.Qz << " " << std::endl;

  if (commands.Fz < RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE)) {
    // For multirotors, disregard yaw commands if throttle is low to prevent motor spin-up while
    // arming/disarming
    commands.Qz = 0.0;
  }

  // Mix the inputs
  float max_output = 1.0;

  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if (mixer_to_use_->output_type[i] != NONE) {
      // Matrix multiply to mix outputs
      outputs_[i] = commands.Fx * mixer_to_use_->Fx[i] +
                    commands.Fy * mixer_to_use_->Fy[i] +
                    commands.Fz * mixer_to_use_->Fz[i] + 
                    commands.Qx * mixer_to_use_->Qx[i] + 
                    commands.Qy * mixer_to_use_->Qy[i] + 
                    commands.Qz * mixer_to_use_->Qz[i];

      // Save off the largest control output if it is greater than 1.0 for future scaling
      if (outputs_[i] > max_output) { max_output = outputs_[i]; }
    }
  }

  return max_output;
}

float Mixer::mix_multirotor_with_motor_parameters()
{
  Controller::Output commands = RF_.controller_.output();

  if (commands.Fz < RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE)
               * RF_.controller_.max_thrust()) {
    // For multirotors, disregard yaw commands if throttle is low to prevent motor spin-up while
    // arming/disarming
    commands.Qz = 0.0;
  }

  // Mix the inputs
  float max_output = 1.0;

  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if (mixer_to_use_->output_type[i] != NONE) {
      // Matrix multiply to mix outputs
      float omega_squared = commands.Fx * mixer_to_use_->Fx[i] +
                            commands.Fy * mixer_to_use_->Fy[i] +
                            commands.Fz * mixer_to_use_->Fz[i] +
                            commands.Qx * mixer_to_use_->Qx[i] +
                            commands.Qy * mixer_to_use_->Qy[i] +
                            commands.Qz * mixer_to_use_->Qz[i];

      // Ensure that omega_squared is non-negative
      if (omega_squared < 0.0) { omega_squared = 0.0; }

      // Ch. 4, setting equation for torque produced by a propeller equal to Eq. 4.19
      // Note that we assume constant airspeed and propeller speed, leading to constant advance ratio,
      // torque, and thrust constants.
      float V_in = rho_ * pow(D_, 5.0) / (4.0 * pow(M_PI, 2.0)) * omega_squared * C_Q_ * R_ / K_Q_
        + R_ * i_0_ + K_V_ * sqrt(omega_squared);

      // Convert desired V_in setting to a throttle setting
      outputs_[i] = V_in / V_max_;

      // Save off the largest control output if it is greater than 1.0 for future scaling
      if (outputs_[i] > max_output) { max_output = outputs_[i]; }
    }
  }

  return max_output;
}

void Mixer::mix_multirotor()
{
  // Mix the outputs based on if a custom mixer (i.e. with motor parameters) is selected.
  float max_output;
  if (mixer_to_use_ == &custom_mixing) {
    max_output = mix_multirotor_with_motor_parameters();
  } else {
    max_output = mix_multirotor_without_motor_parameters();
  }

//  std::cout << "Outputs (pre scale): ";
//  for (auto i : outputs_) {
//    std::cout << i << " ";
//  }
//  std::cout << std::endl;

  // There is no relative scaling on the above equations. In other words, if the input F command is too
  // high, then it will "drown out" all other desired outputs. Therefore, we saturate motor outputs to 
  // maintain controllability even during aggressive maneuvers.
  float scale_factor = 1.0;
  if (max_output > 1.0) { scale_factor = 1.0 / max_output; }

  // Perform Motor Output Scaling
  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    // scale all motor outputs by scale factor (this is usually 1.0, unless we saturated)
    if (mixer_to_use_->output_type[i] == M) { outputs_[i] *= scale_factor; }
  }

 // std::cout << "Outputs (post scale): ";
 // for (auto i : outputs_) {
 //   std::cout << i << " ";
 // }
 // std::cout << std::endl;

}

void Mixer::mix_fixedwing()
{
  Controller::Output commands = RF_.controller_.output();

  // Reverse fixed-wing channels just before mixing if we need to
  if (RF_.params_.get_param_int(PARAM_FIXED_WING)) {
    commands.Qx *= RF_.params_.get_param_int(PARAM_AILERON_REVERSE) ? -1 : 1;
    commands.Qy *= RF_.params_.get_param_int(PARAM_ELEVATOR_REVERSE) ? -1 : 1;
    commands.Qz *= RF_.params_.get_param_int(PARAM_RUDDER_REVERSE) ? -1 : 1;
  }

  // Mix the outputs
  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if (mixer_to_use_->output_type[i] != NONE) {
      // Matrix multiply to mix outputs
      outputs_[i] = commands.Fx * mixer_to_use_->Fx[i] +
                    commands.Qx * mixer_to_use_->Qx[i] +
                    commands.Qy * mixer_to_use_->Qy[i] +
                    commands.Qz * mixer_to_use_->Qz[i];
    }
  }
}

void Mixer::mix_output()
{
  if (mixer_to_use_ == nullptr) { return; }

  // Mix according to airframe type
  if (RF_.params_.get_param_int(PARAM_FIXED_WING)) {
    mix_fixedwing();
  } else {
    mix_multirotor();
  }

  // Insert AUX Commands, and assemble combined_output_types array (Does not override mixer values)

  // For the first NUM_MIXER_OUTPUTS channels, only write aux_command to channels the mixer is not
  // using
  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if (mixer_to_use_->output_type[i] == NONE) {
      outputs_[i] = aux_command_.channel[i].value;
      combined_output_type_[i] = aux_command_.channel[i].type;
    } else {
      combined_output_type_[i] = mixer_to_use_->output_type[i];
    }
  }

  // The other channels are never used by the mixer
  for (uint8_t i = NUM_MIXER_OUTPUTS; i < NUM_TOTAL_OUTPUTS; i++) {
    outputs_[i] = aux_command_.channel[i].value;
    combined_output_type_[i] = aux_command_.channel[i].type;
  }

  // Write to outputs
  for (uint8_t i = 0; i < NUM_TOTAL_OUTPUTS; i++) {
    float value = outputs_[i];
    if (combined_output_type_[i] == S) {
      if (value > 1.0) {
        value = 1.0;
      } else if (value < -1.0) {
        value = -1.0;
      }
      raw_outputs_[i] = value * 0.5 + 0.5;
    } else if (combined_output_type_[i] == M) {
      if (RF_.state_manager_.state().armed) {
        if (value > 1.0) {
          value = 1.0;
        } else if (value < RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE)
                   && RF_.params_.get_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED)) {
          value = RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE);
        } else if (value < 0.0) {
          value = 0.0;
        }
      } else {
        value = 0.0;
      }
      raw_outputs_[i] = value;
    }
  }
  RF_.board_.pwm_write_multi(raw_outputs_, NUM_TOTAL_OUTPUTS);
}

} // namespace rosflight_firmware
