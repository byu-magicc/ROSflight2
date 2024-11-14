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
#include "mavlink.h"

#include "board.h"

#include <cstdint>

namespace rosflight_firmware
{
Mavlink::Mavlink(Board & board)
    : board_(board)
{}

void Mavlink::init(uint32_t baud_rate, uint32_t dev)
{
  board_.serial_init(baud_rate, dev);
  initialized_ = true;
}

//////////////////////////////////////////////////////////////////
//
// Send
//
//////////////////////////////////////////////////////////////////

bool Mavlink::parse_char(uint8_t ch, CommMessage * message)
{
  if (mavlink_parse_char(MAVLINK_COMM_0, ch, &in_buf_, &status_)) {
    return handle_mavlink_message(&in_buf_, message);
  }
  return false;
}

// Rx Message Handlers
bool Mavlink::handle_mavlink_message(const mavlink_message_t * const msg, CommMessage * message)
{
  bool found = true;
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_OFFBOARD_CONTROL:
      handle_msg_offboard_control(&in_buf_, message);
      break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
      handle_msg_param_request_list(&in_buf_, message);
      break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
      handle_msg_param_request_read(&in_buf_, message);
      break;
    case MAVLINK_MSG_ID_PARAM_SET:
      handle_msg_param_set(&in_buf_, message);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_CMD:
      handle_msg_rosflight_cmd(&in_buf_, message);
      break;
    case MAVLINK_MSG_ID_ROSFLIGHT_AUX_CMD:
      handle_msg_rosflight_aux_cmd(&in_buf_, message);
      break;
    case MAVLINK_MSG_ID_TIMESYNC:
      handle_msg_timesync(&in_buf_, message);
      break;
    case MAVLINK_MSG_ID_EXTERNAL_ATTITUDE:
      handle_msg_external_attitude(&in_buf_, message);
      break;
    case MAVLINK_MSG_ID_HEARTBEAT:
      handle_msg_heartbeat(&in_buf_, message);
      break;
    default:
      message->type = CommMessageType::END;
      found = false;
      break;
  }
  return found;
}

void Mavlink::handle_msg_param_request_list(const mavlink_message_t * const msg,
                                            CommMessage * message)
{
  (void) (msg); // unused
  message->type = CommMessageType::MESSAGE_PARAM_REQUEST_LIST;
}

void Mavlink::handle_msg_param_request_read(const mavlink_message_t * const msg,
                                            CommMessage * message)
{
  message->type = CommMessageType::MESSAGE_PARAM_REQUEST_READ;
  mavlink_param_request_read_t read;
  mavlink_msg_param_request_read_decode(msg, &read);

  message->param_read_.id = read.param_index;
  strncpy(message->param_read_.name, read.param_id,
          Params::PARAMS_NAME_LENGTH < 16 ? Params::PARAMS_NAME_LENGTH : 16);
}

void Mavlink::handle_msg_param_set(const mavlink_message_t * const msg, CommMessage * message)
{
  message->type = CommMessageType::MESSAGE_PARAM_SET;
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode(msg, &set);

  mavlink_param_union_t param;
  param.param_float = set.param_value;
  param.type = set.param_type;

  switch (param.type) {
    case MAV_PARAM_TYPE_INT32:
      message->param_set_.value.ivalue = param.param_int32;
      break;
    case MAV_PARAM_TYPE_REAL32:
      message->param_set_.value.fvalue = param.param_float;
      break;
    default:
      // unsupported parameter type
      break;
  }
}

void Mavlink::handle_msg_rosflight_cmd(const mavlink_message_t * const msg, CommMessage * message)
{
  message->type = CommMessageType::MESSAGE_ROSFLIGHT_CMD;
  mavlink_rosflight_cmd_t cmd;
  mavlink_msg_rosflight_cmd_decode(msg, &cmd);

  message->rosflight_cmd_.command = cast_in_range(cmd.command, CommMessageCommand);
}

void Mavlink::handle_msg_rosflight_aux_cmd(const mavlink_message_t * const msg,
                                           CommMessage * message)
{
  message->type = CommMessageType::MESSAGE_ROSFLIGHT_AUX_CMD;
  mavlink_rosflight_aux_cmd_t cmd;
  mavlink_msg_rosflight_aux_cmd_decode(msg, &cmd);

  int len = Mixer::NUM_TOTAL_OUTPUTS < 14 ? Mixer::NUM_TOTAL_OUTPUTS : 14;

  for (int i = 0; i < len; i++) {
    message->new_aux_command_.channel[i].value = cmd.aux_cmd_array[i];

    switch (cast_in_range(cmd.type_array[i], RosflightAuxCmdType)) {
      case RosflightAuxCmdType::SERVO:
        message->new_aux_command_.channel[i].type = Mixer::S;
        break;
      case RosflightAuxCmdType::MOTOR:
        message->new_aux_command_.channel[i].type = Mixer::M;
        break;
      case RosflightAuxCmdType::DISABLED:
      default:
        message->new_aux_command_.channel[i].type = Mixer::NONE;
        message->new_aux_command_.channel[i].value = 0;
        break;
    }
  }
}

void Mavlink::handle_msg_timesync(const mavlink_message_t * const msg, CommMessage * message)
{
  message->type = CommMessageType::MESSAGE_TIMESYNC;
  mavlink_timesync_t tsync;
  mavlink_msg_timesync_decode(msg, &tsync);

  message->time_sync_.local = tsync.tc1;  // client
  message->time_sync_.remote = tsync.ts1; // server
}

void Mavlink::handle_msg_offboard_control(const mavlink_message_t * const msg,
                                          CommMessage * message)
{
  message->type = CommMessageType::MESSAGE_OFFBOARD_CONTROL;
  mavlink_offboard_control_t ctrl;
  mavlink_msg_offboard_control_decode(msg, &ctrl);

  message->offboard_control_.mode = cast_in_range(ctrl.mode, OffboardControlMode);

  message->offboard_control_.U[0].value = ctrl.x;
  message->offboard_control_.U[1].value = ctrl.y;
  message->offboard_control_.U[2].value = ctrl.z;
  message->offboard_control_.U[3].value = ctrl.F;
  message->offboard_control_.U[4].value = 0;
  message->offboard_control_.U[5].value = 0;

  message->offboard_control_.U[0].valid = !(ctrl.ignore & 0x01);
  message->offboard_control_.U[1].valid = !(ctrl.ignore & 0x02);
  message->offboard_control_.U[2].valid = !(ctrl.ignore & 0x04);
  message->offboard_control_.U[3].valid = !(ctrl.ignore & 0x08);
  message->offboard_control_.U[4].valid = false;
  message->offboard_control_.U[5].valid = false;
}

void Mavlink::handle_msg_external_attitude(const mavlink_message_t * const msg,
                                           CommMessage * message)
{
  message->type = CommMessageType::MESSAGE_EXTERNAL_ATTITUDE;
  mavlink_external_attitude_t q_msg;
  mavlink_msg_external_attitude_decode(msg, &q_msg);

  message->external_attitude_quaternion_.q[0] = q_msg.qw;
  message->external_attitude_quaternion_.q[1] = q_msg.qx;
  message->external_attitude_quaternion_.q[2] = q_msg.qy;
  message->external_attitude_quaternion_.q[3] = q_msg.qz;
}

void Mavlink::handle_msg_heartbeat(const mavlink_message_t * const msg, CommMessage * message)
{
  (void) (msg); // unused
  message->type = CommMessageType::MESSAGE_HEARTBEAT;
}

//////////////////////////////////////////////////////////////////
//
// Send
//
//////////////////////////////////////////////////////////////////

void Mavlink::send_attitude_quaternion(uint8_t system_id, const AttitudeStruct & attitude)
{
  mavlink_message_t msg;
  mavlink_msg_attitude_quaternion_pack(system_id, compid_, &msg, attitude.timestamp / 1000,
                                       attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3],
                                       attitude.rate[0], attitude.rate[1], attitude.rate[2]);
  send_message(msg);
}

void Mavlink::send_baro(uint8_t system_id, const PressureStruct & baro)
{
  mavlink_message_t msg;
  mavlink_msg_small_baro_pack(system_id, compid_, &msg, baro.altitude, baro.pressure,
                              baro.temperature);
  send_message(msg);
}

void Mavlink::send_command_ack(uint8_t system_id, uint64_t timestamp_us,
                               CommMessageCommand rosflight_cmd, RosflightCmdResponse success)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_ack_pack(
    system_id, compid_, &msg, static_cast<uint8_t>(rosflight_cmd),
    (success == RosflightCmdResponse::ROSFLIGHT_CMD_SUCCESS)
      ? static_cast<uint8_t>(RosflightCmdResponse::ROSFLIGHT_CMD_SUCCESS)
      : static_cast<uint8_t>(RosflightCmdResponse::ROSFLIGHT_CMD_FAILED));
  send_message(msg);
}

void Mavlink::send_diff_pressure(uint8_t system_id, const PressureStruct & p)
{

  float ias = (p.pressure<0.0?-1.0:1.0)*sqrt(fabs(p.pressure) / (0.5 * 1.225));
  mavlink_message_t msg;
  mavlink_msg_diff_pressure_pack(system_id, compid_, &msg, ias,
                                 p.pressure, p.temperature);
  send_message(msg);
}

void Mavlink::send_heartbeat(uint8_t system_id, uint64_t timestamp_us, bool fixed_wing)
{
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(system_id, compid_, &msg,
                             fixed_wing ? MAV_TYPE_FIXED_WING : MAV_TYPE_QUADROTOR, 0, 0, 0, 0);
  send_message(msg);
}

void Mavlink::send_imu(uint8_t system_id, const ImuStruct & imu)
{
  mavlink_message_t msg;
  mavlink_msg_small_imu_pack(system_id, compid_, &msg, imu.timestamp, imu.accel[0], imu.accel[1],
                             imu.accel[2], imu.gyro[0], imu.gyro[1], imu.gyro[2], imu.temperature);
  send_message(msg, 0);
}

void Mavlink::send_gnss(uint8_t system_id, const GnssStruct & gnss)
{
  {
    mavlink_message_t msg = {};
    mavlink_msg_rosflight_gnss_pack(
      system_id, compid_, &msg, gnss.time_of_week, gnss.fix_type, gnss.time, gnss.nano, gnss.lat,
      gnss.lon, gnss.height_msl, gnss.vel_n, gnss.vel_e, gnss.vel_d, gnss.h_acc, gnss.v_acc,
      gnss.ecef.x, gnss.ecef.y, gnss.ecef.z, gnss.ecef.p_acc, gnss.ecef.vx, gnss.ecef.vy,
      gnss.ecef.vz, gnss.ecef.s_acc, gnss.timestamp);
    send_message(msg);
  }

  {
    mavlink_message_t msg = {};
    mavlink_rosflight_gnss_full_t data = {};
    data.time_of_week = gnss.time_of_week;
    data.year = gnss.year;
    data.month = gnss.month;
    data.day = gnss.day;
    data.hour = gnss.hour;
    data.min = gnss.min;
    data.sec = gnss.sec;
    data.valid = gnss.valid;
    data.t_acc = gnss.t_acc;
    data.nano = gnss.nano;
    data.fix_type = gnss.fix_type;
    data.num_sat = gnss.num_sat;
    data.lon = gnss.lon;
    data.lat = gnss.lat;
    data.height = gnss.height_ellipsoid;
    data.height_msl = gnss.height_msl;
    data.h_acc = gnss.h_acc;
    data.v_acc = gnss.v_acc;
    data.vel_n = gnss.vel_n;
    data.vel_e = gnss.vel_e;
    data.vel_d = gnss.vel_d;
    data.g_speed = gnss.ground_speed;
    data.head_mot = gnss.course;
    data.s_acc = gnss.speed_accy;
    data.head_acc = gnss.course_accy;
    data.p_dop = gnss.dop;
    data.rosflight_timestamp = gnss.timestamp;
    mavlink_msg_rosflight_gnss_full_encode(system_id, compid_, &msg, &data);
    send_message(msg);
  }
}

void Mavlink::send_log_message(uint8_t system_id, uint64_t timestamp_us, LogSeverity severity,
                               const char * text)
{
  MAV_SEVERITY mavlink_severity = MAV_SEVERITY_ENUM_END;
  switch (severity) {
    case CommLinkInterface::LogSeverity::LOG_INFO:
      mavlink_severity = MAV_SEVERITY_INFO;
      break;
    case CommLinkInterface::LogSeverity::LOG_WARNING:
      mavlink_severity = MAV_SEVERITY_WARNING;
      break;
    case CommLinkInterface::LogSeverity::LOG_ERROR:
      mavlink_severity = MAV_SEVERITY_ERROR;
      break;
    case CommLinkInterface::LogSeverity::LOG_CRITICAL:
      mavlink_severity = MAV_SEVERITY_CRITICAL;
      break;
    default:
      break;
  }

  mavlink_message_t msg;
  mavlink_msg_statustext_pack(system_id, compid_, &msg, static_cast<uint8_t>(mavlink_severity),
                              text);
  send_message(msg);
}

void Mavlink::send_mag(uint8_t system_id, const MagStruct & mag)
{
  mavlink_message_t msg;
  mavlink_msg_small_mag_pack(system_id, compid_, &msg, mag.flux[0], mag.flux[1], mag.flux[2]);
  send_message(msg);
}

void Mavlink::send_named_value_int(uint8_t system_id, uint64_t timestamp_us,
                                   const char * const name, int32_t value)
{
  uint32_t timestamp_ms = timestamp_us / 1000;
  mavlink_message_t msg;
  mavlink_msg_named_value_int_pack(system_id, compid_, &msg, timestamp_ms, name, value);
  send_message(msg);
}

void Mavlink::send_named_value_float(uint8_t system_id, uint64_t timestamp_us,
                                     const char * const name, float value)
{
  uint32_t timestamp_ms = timestamp_us / 1000;
  mavlink_message_t msg;
  mavlink_msg_named_value_float_pack(system_id, compid_, &msg, timestamp_ms, name, value);
  send_message(msg);
}

void Mavlink::send_output_raw(uint8_t system_id, const RcStruct & raw)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_output_raw_pack(system_id, compid_, &msg, raw.timestamp, raw.chan);
  send_message(msg);
}

void Mavlink::send_param_value_int(uint8_t system_id, uint64_t timestamp_us, uint16_t index,
                                   const char * const name, int32_t value, uint16_t param_count)
{
  mavlink_param_union_t param;
  param.param_int32 = value;

  mavlink_message_t msg;
  mavlink_msg_param_value_pack(system_id, 0, &msg, name, param.param_float, MAV_PARAM_TYPE_INT32,
                               param_count, index);
  send_message(msg);
}

void Mavlink::send_param_value_float(uint8_t system_id, uint64_t timestamp_us, uint16_t index,
                                     const char * const name, float value, uint16_t param_count)
{
  mavlink_message_t msg;
  mavlink_msg_param_value_pack(system_id, 0, &msg, name, value, MAV_PARAM_TYPE_REAL32, param_count,
                               index);
  send_message(msg);
}

void Mavlink::send_rc_raw(uint8_t system_id, const RcStruct & rc)
{
  mavlink_message_t msg;
  mavlink_msg_rc_channels_pack(
    system_id, compid_, &msg, rc.timestamp / 1000, 0, rc.chan[0] * 1000 + 1000,
    rc.chan[1] * 1000 + 1000, rc.chan[2] * 1000 + 1000, rc.chan[3] * 1000 + 1000,
    rc.chan[4] * 1000 + 1000, rc.chan[5] * 1000 + 1000, rc.chan[6] * 1000 + 1000,
    rc.chan[7] * 1000 + 1000, rc.chan[8] * 1000 + 1000, rc.chan[9] * 1000 + 1000,
    rc.chan[10] * 1000 + 1000, rc.chan[11] * 1000 + 1000, rc.chan[12] * 1000 + 1000,
    rc.chan[13] * 1000 + 1000, rc.chan[14] * 1000 + 1000, rc.chan[15] * 1000 + 1000,
    rc.chan[16] * 1000 + 1000, rc.nChan, 0);
  send_message(msg);
}

void Mavlink::send_sonar(uint8_t system_id, const RangeStruct & range)
{
  mavlink_message_t msg;
  mavlink_msg_small_range_pack(system_id, compid_, &msg,
                               static_cast<uint8_t>(RosFlightRangeType::ROSFLIGHT_RANGE_SONAR),
                               range.range, range.max_range, range.min_range);
  send_message(msg);
}

void Mavlink::send_status(uint8_t system_id, uint64_t timestamp_us, bool armed, bool failsafe,
                          bool rc_override, bool offboard, uint8_t error_code, uint8_t control_mode,
                          int16_t num_errors, int16_t loop_time_us)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_status_pack(system_id, compid_, &msg, armed, failsafe, rc_override,
                                    offboard, error_code, control_mode, num_errors, loop_time_us);
  send_message(msg);
}

void Mavlink::send_timesync(uint8_t system_id, uint64_t timestamp_us, int64_t tc1, int64_t ts1)
{
  mavlink_message_t msg;
  mavlink_msg_timesync_pack(system_id, compid_, &msg, tc1, ts1);
  send_message(msg, 1);
}

void Mavlink::send_version(uint8_t system_id, uint64_t timestamp_us, const char * const version)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_version_pack(system_id, compid_, &msg, version);
  send_message(msg);
}

void Mavlink::send_error_data(uint8_t system_id, uint64_t timestamp_us,
                              const StateManager::BackupData & error_data)
{
  mavlink_message_t msg;
  bool rearm = (error_data.arm_flag == StateManager::BackupData::ARM_MAGIC);
  mavlink_msg_rosflight_hard_error_pack(system_id, compid_, &msg, error_data.error_code,
                                        error_data.debug.pc, error_data.reset_count, rearm);
  send_message(msg);
}

void Mavlink::send_battery_status(uint8_t system_id, const BatteryStruct & batt)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_battery_status_pack(system_id, compid_, &msg, batt.voltage, batt.current);
  send_message(msg);
}

void Mavlink::send_message(const mavlink_message_t & msg, uint8_t qos)
{
  if (initialized_) {
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(data, &msg);
    board_.serial_write(data, len, qos);
  }
}

} // namespace rosflight_firmware
