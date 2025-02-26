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

#ifndef ROSFLIGHT_FIRMWARE_MAVLINK_H
#define ROSFLIGHT_FIRMWARE_MAVLINK_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "v1.0/rosflight/mavlink.h"
#pragma GCC diagnostic pop

#include "interface/comm_link.h"

#include "board.h"

namespace rosflight_firmware
{
class Board;

class Mavlink : public CommLinkInterface
{
public:
  Mavlink(Board & board);
  void init(uint32_t baud_rate, uint32_t dev) override;

  bool parse_char(uint8_t ch, CommMessage * message) override;

  void send_attitude_quaternion(const AttitudeStruct & attitude) override;

  void send_baro(const PressureStruct & baro) override;

  void send_command_ack(uint64_t timestamp_us, CommMessageCommand command,
                        RosflightCmdResponse success) override;

  void send_diff_pressure(const PressureStruct & p) override;

  void send_heartbeat(uint64_t timestamp_us, bool fixed_wing) override;

  void send_imu(const ImuStruct & imu) override;

  void send_log_message(uint64_t timestamp_us, LogSeverity severity,
                        const char * text) override;

  void send_mag(const MagStruct & mag) override;

//  void send_named_value_int(uint64_t timestamp_us, const char * const name,
//                            int32_t value) override;
//
//  void send_named_value_float(uint64_t imestamp_us, const char * const name,
//                              float value) override;

  void send_output_raw(const RcStruct & raw) override;

  void send_param_value_int(uint64_t timestamp_us, uint16_t index,
                            const char * const name, int32_t value, uint16_t param_count) override;

  void send_param_value_float(uint64_t timestamp_us, uint16_t index,
                              const char * const name, float value, uint16_t param_count) override;

  void send_rc_raw(const RcStruct & rc) override;

  void send_sonar(const RangeStruct & sonar) override;

  void send_status(uint64_t timestamp_us, bool armed, bool failsafe,
                   bool rc_override, bool offboard, uint8_t error_code, uint8_t control_mode,
                   int16_t num_errors, int16_t loop_time_us) override;

  void send_timesync(uint64_t timestamp_us, int64_t tc1, int64_t ts1) override;

  void send_version(uint64_t timestamp_us, const char * const version) override;

  void send_gnss(const GnssStruct & gnss) override;

  void send_error_data(uint64_t timestamp_us,
                       const StateManager::BackupData & error_data) override;

  void send_battery_status(const BatteryStruct & batt) override;

private:
  void send_message(const mavlink_message_t & msg, uint8_t qos = UINT8_MAX);

  void handle_msg_param_request_list(const mavlink_message_t * const msg, CommMessage * message);
  void handle_msg_param_request_read(const mavlink_message_t * const msg, CommMessage * message);
  void handle_msg_param_set(const mavlink_message_t * const msg, CommMessage * message);
  void handle_msg_offboard_control(const mavlink_message_t * const msg, CommMessage * message);
  void handle_msg_offboard_control_full(const mavlink_message_t * const msg, CommMessage * message);
  void handle_msg_external_attitude(const mavlink_message_t * const msg, CommMessage * message);
  void handle_msg_rosflight_cmd(const mavlink_message_t * const msg, CommMessage * message);
  void handle_msg_rosflight_aux_cmd(const mavlink_message_t * const msg, CommMessage * message);
  void handle_msg_timesync(const mavlink_message_t * const msg, CommMessage * message);
  void handle_msg_heartbeat(const mavlink_message_t * const msg, CommMessage * message);
  bool handle_mavlink_message(const mavlink_message_t * const msg, CommMessage * message);

  Board & board_;
  uint8_t system_id_;

  uint32_t compid_ = 250;
  mavlink_message_t in_buf_;
  mavlink_status_t status_;
  bool initialized_ = false;
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_MAVLINK_H
