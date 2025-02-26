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

#include "rc.h"

#include "rosflight.h"

#include <cstring>

namespace rosflight_firmware
{
RC::RC(ROSflight & _rf)
    : RF_(_rf)
{}

void RC::init()
{
  init_rc();
  new_command_ = false;
}

void RC::init_rc()
{
  RF_.board_.rc_init(static_cast<Board::rc_type_t>(RF_.params_.get_param_int(PARAM_RC_TYPE)));
  init_sticks();
  init_switches();
}

RcStruct * RC::get_rc(void) { return &rc_; }
float RC::read_chan(uint8_t chan) { return rc_.chan[chan]; }

bool RC::receive(void) { return RF_.board_.rc_read(&rc_); }

void RC::param_change_callback(uint16_t param_id)
{
  switch (param_id) {
    case PARAM_RC_TYPE:
      RF_.board_.rc_init(static_cast<Board::rc_type_t>(RF_.params_.get_param_int(PARAM_RC_TYPE)));
      break;
    case PARAM_RC_X_CHANNEL:
    case PARAM_RC_Y_CHANNEL:
    case PARAM_RC_Z_CHANNEL:
    case PARAM_RC_F_CHANNEL:
      init_sticks();
      break;
    case PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL:
    case PARAM_RC_THROTTLE_OVERRIDE_CHANNEL:
    case PARAM_RC_ATT_CONTROL_TYPE_CHANNEL:
    case PARAM_RC_ARM_CHANNEL:
    case PARAM_RC_SWITCH_5_DIRECTION:
    case PARAM_RC_SWITCH_6_DIRECTION:
    case PARAM_RC_SWITCH_7_DIRECTION:
    case PARAM_RC_SWITCH_8_DIRECTION:
      init_switches();
      break;
    default:
      // do nothing
      break;
  }
}

float RC::stick(Stick channel) { return stick_values[channel]; }

bool RC::switch_on(Switch channel) { return switch_values[channel]; }

bool RC::switch_mapped(Switch channel) { return switches[channel].mapped; }

void RC::init_sticks(void)
{
  sticks[STICK_X].channel = RF_.params_.get_param_int(PARAM_RC_X_CHANNEL);
  sticks[STICK_X].one_sided = false;

  sticks[STICK_Y].channel = RF_.params_.get_param_int(PARAM_RC_Y_CHANNEL);
  sticks[STICK_Y].one_sided = false;

  sticks[STICK_Z].channel = RF_.params_.get_param_int(PARAM_RC_Z_CHANNEL);
  sticks[STICK_Z].one_sided = false;

  sticks[STICK_F].channel = RF_.params_.get_param_int(PARAM_RC_F_CHANNEL);
  sticks[STICK_F].one_sided = true;
}

void RC::init_switches()
{
  for (uint8_t chan = 0; chan < static_cast<uint8_t>(SWITCHES_COUNT); chan++) {
    char channel_name[18];
    switch (chan) {
      case SWITCH_ARM:
        strcpy(channel_name, "ARM");
        switches[chan].channel = RF_.params_.get_param_int(PARAM_RC_ARM_CHANNEL);
        break;
      case SWITCH_ATT_OVERRIDE:
        strcpy(channel_name, "ATTITUDE OVERRIDE");
        switches[chan].channel = RF_.params_.get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL);
        break;
      case SWITCH_THROTTLE_OVERRIDE:
        strcpy(channel_name, "THROTTLE OVERRIDE");
        switches[chan].channel = RF_.params_.get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL);
        break;
      case SWITCH_ATT_TYPE:
        strcpy(channel_name, "ATTITUDE TYPE");
        switches[chan].channel = RF_.params_.get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL);
        break;
      default:
        strcpy(channel_name, "INVALID");
        switches[chan].channel = 255;
        break;
    }

    switches[chan].mapped = switches[chan].channel > 3
      && switches[chan].channel < RF_.params_.get_param_int(PARAM_RC_NUM_CHANNELS);

    switch (switches[chan].channel) {
      case 4:
        switches[chan].direction = RF_.params_.get_param_int(PARAM_RC_SWITCH_5_DIRECTION);
        break;
      case 5:
        switches[chan].direction = RF_.params_.get_param_int(PARAM_RC_SWITCH_6_DIRECTION);
        break;
      case 6:
        switches[chan].direction = RF_.params_.get_param_int(PARAM_RC_SWITCH_7_DIRECTION);
        break;
      case 7:
        switches[chan].direction = RF_.params_.get_param_int(PARAM_RC_SWITCH_8_DIRECTION);
        break;
      default:
        switches[chan].direction = 1;
        break;
    }

    if (switches[chan].mapped) {
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,
                            "%s switch mapped to RC channel %d", channel_name,
                            switches[chan].channel);
    } else {
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO, "%s switch not mapped",
                            channel_name);
    }
  }
}

bool RC::check_rc_lost()
{
  bool failsafe = false;

  // If the board reports that we have lost RC, tell the state manager
  if (rc_.frameLost | rc_.failsafeActivated) {
    failsafe = true;
  } else {
    // go into failsafe if we get an invalid RC command for any channel
    for (int8_t i = 0; i < RF_.params_.get_param_int(PARAM_RC_NUM_CHANNELS); i++) {
      float pwm = read_chan(i);
      if (pwm < -0.25 || pwm > 1.25) { failsafe = true; }
    }
  }

  if (failsafe) {
    // set the RC Lost error flag
    RF_.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  } else {
    // Clear the RC Lost Error
    RF_.state_manager_.set_event(StateManager::EVENT_RC_FOUND);
  }

  return failsafe;
}

void RC::look_for_arm_disarm_signal()
{
  uint32_t now_ms = RF_.board_.clock_millis();
  uint32_t dt = now_ms - prev_time_ms;
  prev_time_ms = now_ms;
  // check for arming switch
  if (!switch_mapped(SWITCH_ARM)) {
    if (!RF_.state_manager_.state().armed) { // we are DISARMED
      // if left stick is down and to the right
      if ((stick(STICK_F) < RF_.params_.get_param_float(PARAM_ARM_THRESHOLD))
          && (stick(STICK_Z) > (1.0f - RF_.params_.get_param_float(PARAM_ARM_THRESHOLD)))) {
        time_sticks_have_been_in_arming_position_ms += dt;
      } else {
        time_sticks_have_been_in_arming_position_ms = 0;
      }
      if (time_sticks_have_been_in_arming_position_ms > 1000) {
        RF_.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
      }
    } else { // we are ARMED
      // if left stick is down and to the left
      if (stick(STICK_F) < RF_.params_.get_param_float(PARAM_ARM_THRESHOLD)
          && stick(STICK_Z) < -(1.0f - RF_.params_.get_param_float(PARAM_ARM_THRESHOLD))) {
        time_sticks_have_been_in_arming_position_ms += dt;
      } else {
        time_sticks_have_been_in_arming_position_ms = 0;
      }
      if (time_sticks_have_been_in_arming_position_ms > 1000) {
        RF_.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
        time_sticks_have_been_in_arming_position_ms = 0;
      }
    }
  } else { // ARMING WITH SWITCH
    if (switch_on(SWITCH_ARM)) {
      if (!RF_.state_manager_.state().armed) {
        RF_.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
      }
    } else {
      RF_.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
    }
  }
}

bool RC::run()
{
  // Check for rc lost
  if (check_rc_lost()) { return false; }

  // read and normalize stick values
  for (uint8_t channel = 0; channel < static_cast<uint8_t>(STICKS_COUNT); channel++) {
    float pwm = read_chan(sticks[channel].channel);
    if (sticks[channel].one_sided) { // generally only F is one_sided
      stick_values[channel] = pwm;
    } else {
      stick_values[channel] = 2.0 * (pwm - 0.5);
    }
  }

  // read and interpret switch values
  for (uint8_t channel = 0; channel < static_cast<uint8_t>(SWITCHES_COUNT); channel++) {
    if (switches[channel].mapped) {
      float pwm = read_chan(switches[channel].channel);
      if (switches[channel].direction < 0) {
        switch_values[channel] = pwm < 0.2;
      } else {
        switch_values[channel] = pwm >= 0.8;
      }
    } else {
      switch_values[channel] = false;
    }
  }

  // Look for arming and disarming signals
  look_for_arm_disarm_signal();

  // Signal to the mux that we need to compute a new combined command
  new_command_ = true;
  return true;
}

bool RC::new_command()
{
  if (new_command_) {
    new_command_ = false;
    return true;
  } else {
    return false;
  }
}

uint16_t RC::fake_rx(uint16_t * chan, uint16_t len, bool lost, bool failsafe)
{
  len = (len < RC_STRUCT_CHANNELS) ? len : RC_STRUCT_CHANNELS;
  for (int i = 0; i < len; i++) rc_.chan[i] = (static_cast<float>(chan[i]) - 1000.) / 1000.;
  rc_.failsafeActivated = failsafe;
  rc_.frameLost = lost;
  rc_.nChan = RC_STRUCT_CHANNELS;
  rc_.header.timestamp = RF_.board_.clock_micros();
  rc_.header.status = failsafe | lost;
  run();
  return len;
};

} // namespace rosflight_firmware
