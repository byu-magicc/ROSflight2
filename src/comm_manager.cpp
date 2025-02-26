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

#include "comm_manager.h"

#include "param.h"

#include "rosflight.h"

#include <cstdint>
#include <cstring>

namespace rosflight_firmware
{
CommManager::LogMessageBuffer::LogMessageBuffer() { memset(buffer_, 0, sizeof(buffer_)); }

void CommManager::LogMessageBuffer::add_message(CommLinkInterface::LogSeverity severity, char msg[])
{
  LogMessage & newest_msg = buffer_[newest_];
  strcpy(newest_msg.msg, msg);
  newest_msg.severity = severity;

  newest_ = (newest_ + 1) % LOG_BUF_SIZE;

  // quietly over-write old messages (what else can we do?)
  length_ += 1;
  if (length_ > LOG_BUF_SIZE) {
    length_ = LOG_BUF_SIZE;
    oldest_ = (oldest_ + 1) % LOG_BUF_SIZE;
  }
}

void CommManager::LogMessageBuffer::pop()
{
  if (length_ > 0) {
    length_--;
    oldest_ = (oldest_ + 1) % LOG_BUF_SIZE;
  }
}

CommManager::CommManager(ROSflight & rf, CommLinkInterface & comm_link)
    : RF_(rf)
    , comm_link_(comm_link)
{}

void CommManager::init()
{
  comm_link_.init(static_cast<uint32_t>(RF_.params_.get_param_int(PARAM_BAUD_RATE)),
                  static_cast<uint32_t>(RF_.params_.get_param_int(PARAM_SERIAL_DEVICE)));

  offboard_control_time_ = 0;
  send_params_index_ = PARAMS_COUNT;

  update_system_id(PARAM_SYSTEM_ID);

  initialized_ = true;
}

void CommManager::param_change_callback(uint16_t param_id)
{
  switch (param_id) {
    case PARAM_SYSTEM_ID:
      update_system_id(param_id);
      break;
    default:
      // do nothing
      break;
  }
}

void CommManager::update_system_id(uint16_t param_id)
{
  sysid_ = static_cast<uint8_t>(RF_.params_.get_param_int(param_id));
}

void CommManager::update_status() { send_status(); }

void CommManager::send_param_value(uint16_t param_id)
{
  if (param_id < PARAMS_COUNT) {
    switch (RF_.params_.get_param_type(param_id)) {
      case PARAM_TYPE_INT32:
        comm_link_.send_param_value_int(
          RF_.board_.clock_micros(), param_id, RF_.params_.get_param_name(param_id),
          RF_.params_.get_param_int(param_id), static_cast<uint16_t>(PARAMS_COUNT));
        break;
      case PARAM_TYPE_FLOAT:
        comm_link_.send_param_value_float(
          RF_.board_.clock_micros(), param_id, RF_.params_.get_param_name(param_id),
          RF_.params_.get_param_float(param_id), static_cast<uint16_t>(PARAMS_COUNT));
        break;
      default:
        break;
    }
  }
}

void CommManager::log(CommLinkInterface::LogSeverity severity, const char * fmt, ...)
{
  // Convert the format string to a raw char array
  va_list args;
  va_start(args, fmt);
  char message[LOG_MSG_SIZE];
  vsnprintf(message, LOG_MSG_SIZE, fmt, args);
  va_end(args);

  log_message(severity, message);
}

void CommManager::log_message(CommLinkInterface::LogSeverity severity, char * text)
{
  if (initialized_ && connected_) {
    comm_link_.send_log_message(RF_.board_.clock_micros(), severity, text);
  } else {
    log_buffer_.add_message(severity, text);
  }
}

void CommManager::send_heartbeat(void)
{
  comm_link_.send_heartbeat(RF_.board_.clock_micros(),
                            static_cast<bool>(RF_.params_.get_param_int(PARAM_FIXED_WING)));
}

void CommManager::send_status(void)
{
  if (!initialized_) { return; }

  uint8_t control_mode = 0;
  if (RF_.params_.get_param_int(PARAM_FIXED_WING)
      || RF_.command_manager_.combined_control().x.type == PASSTHROUGH) {
    control_mode = MODE_PASS_THROUGH;
  } else if (RF_.command_manager_.combined_control().x.type == ANGLE) {
    control_mode = MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  } else {
    control_mode = MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  }

  comm_link_.send_status(
    RF_.board_.clock_micros(), RF_.state_manager_.state().armed,
    RF_.state_manager_.state().failsafe, RF_.command_manager_.rc_override_active(),
    RF_.command_manager_.offboard_control_active(), RF_.state_manager_.state().error_codes,
    control_mode, RF_.board_.sensors_errors_count(), RF_.get_loop_time_us());
}

void CommManager::send_attitude(void)
{
  comm_link_.send_attitude_quaternion(*RF_.estimator_.get_attitude());
}

void CommManager::send_imu(void) { comm_link_.send_imu(*RF_.sensors_.get_imu()); }

void CommManager::send_output_raw(void)
{
  comm_link_.send_output_raw(*RF_.mixer_.get_output_raw());
}

void CommManager::send_rc_raw(void) { comm_link_.send_rc_raw(*RF_.rc_.get_rc()); }

void CommManager::send_diff_pressure(void)
{
  comm_link_.send_diff_pressure(*RF_.sensors_.get_diff_pressure());
}

void CommManager::send_baro(void) { comm_link_.send_baro(*RF_.sensors_.get_baro()); }

void CommManager::send_sonar(void) { comm_link_.send_sonar(*RF_.sensors_.get_sonar()); }

void CommManager::send_mag(void) { comm_link_.send_mag(*RF_.sensors_.get_mag()); }

void CommManager::send_battery_status(void)
{
  comm_link_.send_battery_status(*RF_.sensors_.get_battery());
}

void CommManager::send_backup_data(const StateManager::BackupData & backup_data)
{
  if (connected_) {
    comm_link_.send_error_data(RF_.board_.clock_micros(), backup_data);
  } else {
    backup_data_buffer_ = backup_data;
    have_backup_data_ = true;
  }
}

void CommManager::send_gnss(void) { comm_link_.send_gnss(*RF_.sensors_.get_gnss()); }

//void CommManager::send_named_value_int(const char * const name, int32_t value)
//{
//  comm_link_.send_named_value_int(RF_.board_.clock_micros(), name, value);
//}
//
//void CommManager::send_named_value_float(const char * const name, float value)
//{
//  comm_link_.send_named_value_float(RF_.board_.clock_micros(), name, value);
//}

void CommManager::send_next_param(void)
{
  if (send_params_index_ < PARAMS_COUNT) {
    send_param_value(static_cast<uint16_t>(send_params_index_));
    send_params_index_++;
  }
}

void CommManager::transmit(got_flags got)
{
  uint64_t time_us = RF_.board_.clock_micros();

  // Send out data

  if (got.imu) { // Nominally 400Hz
    send_imu();
    send_attitude();
    static uint64_t ro_count = 0;
    if (!((ro_count++) % 8)) { send_output_raw(); } // Raw output at 400Hz/8 = 50Hz
  }

  // Pitot sensor
  if (got.diff_pressure) { send_diff_pressure(); }
  // Baro altitude
  if (got.baro) { send_baro(); }
  // Magnetometer
  if (got.mag) { send_mag(); }
  // Height above ground sensor (not enabled)
  if (got.sonar) { send_sonar(); }
  // Battery V & I
  if (got.battery) { send_battery_status(); }
  // GPS data (GNSS Packed)
  if (got.gnss) { send_gnss(); }

  {
    static uint64_t next_heartbeat = 0, next_status = 0;

    if ((time_us) / 1000000 >= next_heartbeat) { // 1 Hz
      send_heartbeat();
      next_heartbeat = time_us / 1000000 + 1;
    }
    if ((time_us) / 100000 >= next_status) { // 10 Hz
      send_status();
      next_status = time_us / 100000 + 1;
    }
  }

  send_next_param();

  // send buffered log messages
  if (connected_ && !log_buffer_.empty()) {
    const LogMessageBuffer::LogMessage & msg = log_buffer_.oldest();
    comm_link_.send_log_message(RF_.board_.clock_micros(), msg.severity, msg.msg);
    log_buffer_.pop();
  }
}

void CommManager::receive(void)
{
  CommLinkInterface::CommMessage message;

  //PTT This could hang if there is too much incomming serial data!
  while (RF_.board_.serial_bytes_available()) {
    if (comm_link_.parse_char(RF_.board_.serial_read(), &message)) {
      switch (message.type) {
        case CommLinkInterface::CommMessageType::MESSAGE_OFFBOARD_CONTROL:
          receive_msg_offboard_control(&message);
          break;
        case CommLinkInterface::CommMessageType::MESSAGE_PARAM_REQUEST_LIST:
          receive_msg_param_request_list(&message);
          break;
        case CommLinkInterface::CommMessageType::MESSAGE_PARAM_REQUEST_READ:
          receive_msg_param_request_read(&message);
          break;
        case CommLinkInterface::CommMessageType::MESSAGE_PARAM_SET:
          receive_msg_param_set(&message);
          break;
        case CommLinkInterface::CommMessageType::MESSAGE_ROSFLIGHT_CMD:
          receive_msg_rosflight_cmd(&message);
          break;
        case CommLinkInterface::CommMessageType::MESSAGE_ROSFLIGHT_AUX_CMD:
          receive_msg_rosflight_aux_cmd(&message);
          break;
        case CommLinkInterface::CommMessageType::MESSAGE_TIMESYNC:
          receive_msg_timesync(&message);
          break;
        case CommLinkInterface::CommMessageType::MESSAGE_EXTERNAL_ATTITUDE:
          receive_msg_external_attitude(&message);
          break;
        case CommLinkInterface::CommMessageType::MESSAGE_HEARTBEAT:
          receive_msg_heartbeat(&message);
          break;
        default:
          break; // Message not recognized
      }
    }
  }
}

void CommManager::receive_msg_param_request_list(CommLinkInterface::CommMessage * message)
{
  (void) message; // unused
  send_params_index_ = 0;
}

void CommManager::receive_msg_param_request_read(CommLinkInterface::CommMessage * message)
{
  uint16_t id = (message->param_read_.id < 0)
    ? RF_.params_.lookup_param_id(message->param_read_.name)
    : static_cast<uint16_t>(message->param_read_.id);
  if (id < PARAMS_COUNT) { send_param_value(id); }
}

void CommManager::receive_msg_param_set(CommLinkInterface::CommMessage * message)
{
  uint16_t id = RF_.params_.lookup_param_id(message->param_set_.name);

  if (id < PARAMS_COUNT) {
    if (RF_.params_.get_param_type(id) == PARAM_TYPE_FLOAT) {
      RF_.params_.set_param_float(id, message->param_set_.value.fvalue);
    } else if (RF_.params_.get_param_type(id) == PARAM_TYPE_INT32) {
      RF_.params_.set_param_int(id, message->param_set_.value.ivalue);
    }
  }
}

void CommManager::receive_msg_rosflight_cmd(CommLinkInterface::CommMessage * message)
{
  bool result = true;
  bool reboot_flag = false;
  bool reboot_to_bootloader_flag = false;

  // None of these actions can be performed if we are armed
  if (RF_.state_manager_.state().armed) {
    result = false;
  } else {
    result = true;

    switch (message->rosflight_cmd_.command) {
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_READ_PARAMS:
        result = RF_.params_.read();
        break;
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_WRITE_PARAMS:
        result = RF_.params_.write();
        break;
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_SET_PARAM_DEFAULTS:
        RF_.params_.set_defaults();
        break;
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_ACCEL_CALIBRATION:
        result = RF_.sensors_.start_imu_calibration();
        break;
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_GYRO_CALIBRATION:
        result = RF_.sensors_.start_gyro_calibration();
        break;
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_AIRSPEED_CALIBRATION:
        result = RF_.sensors_.start_diff_pressure_calibration();
        break;
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_RC_CALIBRATION:
        RF_.controller_.calculate_equilbrium_torque_from_rc();
        break;
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_REBOOT:
        reboot_flag = true;
        break;
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_REBOOT_TO_BOOTLOADER:
        reboot_to_bootloader_flag = true;
        break;
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_SEND_VERSION:
        comm_link_.send_version(RF_.board_.clock_micros(), GIT_VERSION_STRING);
        break;
      // Unsupported commands. Report failure.
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_RESET_ORIGIN:
      case CommLinkInterface::CommMessageCommand::ROSFLIGHT_CMD_SEND_ALL_CONFIG_INFOS:
      default:
        result = false;
        log(CommLinkInterface::LogSeverity::LOG_ERROR,
            "Unsupported CommLinkInterface::ROSFLIGHT CMD %d", message->rosflight_cmd_.command);
    }
  }
  CommLinkInterface::RosflightCmdResponse response =
    cast_in_range(result, CommLinkInterface::RosflightCmdResponse);

  comm_link_.send_command_ack(RF_.board_.clock_micros(), message->rosflight_cmd_.command, response);

  if (reboot_flag || reboot_to_bootloader_flag) {
    RF_.board_.clock_delay(20);
    RF_.board_.board_reset(reboot_to_bootloader_flag);
  }
  RF_.board_.serial_flush();
}

void CommManager::receive_msg_timesync(CommLinkInterface::CommMessage * message)
{
  uint64_t now_us = RF_.board_.clock_micros();

  // Respond if this is a request local==0 vs. response local!=0
  if (message->time_sync_.local == 0) {
    comm_link_.send_timesync(RF_.board_.clock_micros(), static_cast<int64_t>(now_us) * 1000,
                             message->time_sync_.remote);
  }
}

void CommManager::receive_msg_offboard_control(CommLinkInterface::CommMessage * message)
{
  // put values into a new command struct
  control_t new_offboard_command;

  switch (message->offboard_control_.mode) {
    case CommLinkInterface::OffboardControlMode::MODE_PASS_THROUGH:
      new_offboard_command.x.type = PASSTHROUGH;
      new_offboard_command.y.type = PASSTHROUGH;
      new_offboard_command.z.type = PASSTHROUGH;
      new_offboard_command.F.type = THROTTLE;
      break;
    case CommLinkInterface::OffboardControlMode::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
      new_offboard_command.x.type = RATE;
      new_offboard_command.y.type = RATE;
      new_offboard_command.z.type = RATE;
      new_offboard_command.F.type = THROTTLE;
      break;
    case CommLinkInterface::OffboardControlMode::MODE_ROLL_PITCH_YAWRATE_THROTTLE:
      new_offboard_command.x.type = ANGLE;
      new_offboard_command.y.type = ANGLE;
      new_offboard_command.z.type = RATE;
      new_offboard_command.F.type = THROTTLE;
      break;
    case CommLinkInterface::OffboardControlMode::MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
    case CommLinkInterface::OffboardControlMode::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
    case CommLinkInterface::OffboardControlMode::MODE_XPOS_YPOS_YAW_ALTITUDE:
    default:
      // invalid mode; ignore message and return without calling callback
      return;
  }

  new_offboard_command.x.value = message->offboard_control_.U[0].value;
  new_offboard_command.y.value = message->offboard_control_.U[1].value;
  new_offboard_command.z.value = message->offboard_control_.U[2].value;
  new_offboard_command.F.value = message->offboard_control_.U[3].value;

  new_offboard_command.x.active = !(message->offboard_control_.U[0].valid & 0x01);
  new_offboard_command.y.active = !(message->offboard_control_.U[1].valid & 0x02);
  new_offboard_command.z.active = !(message->offboard_control_.U[2].valid & 0x04);
  new_offboard_command.F.active = !(message->offboard_control_.U[3].valid & 0x08);

  // Tell the command_manager that we have a new command we need to mux
  new_offboard_command.stamp_ms = RF_.board_.clock_millis();
  RF_.command_manager_.set_new_offboard_command(new_offboard_command);
}

void CommManager::receive_msg_rosflight_aux_cmd(CommLinkInterface::CommMessage * message)
{
  RF_.mixer_.set_new_aux_command(message->new_aux_command_);
}

void CommManager::receive_msg_external_attitude(CommLinkInterface::CommMessage * message)
{
  turbomath::Quaternion q;
  q.w = message->external_attitude_quaternion_.q[0];
  q.x = message->external_attitude_quaternion_.q[1];
  q.y = message->external_attitude_quaternion_.q[2];
  q.z = message->external_attitude_quaternion_.q[3];

  RF_.estimator_.set_external_attitude_update(q);
}

void CommManager::receive_msg_heartbeat(CommLinkInterface::CommMessage * message)
{
  (void) message; // unused
  connected_ = true;

  // PTT consider moving this to CommManager::send_backup_data
  if (have_backup_data_) {
    comm_link_.send_error_data(RF_.board_.clock_micros(), backup_data_buffer_);
    have_backup_data_ = false;
  }
}

} // namespace rosflight_firmware
