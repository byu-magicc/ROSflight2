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

#ifndef ROSFLIGHT_FIRMWARE_COMM_MANAGER_H
#define ROSFLIGHT_FIRMWARE_COMM_MANAGER_H

#include "interface/comm_link.h"
#include "interface/param_listener.h"

#include <stdarg.h>
#include <stdio.h>

#include <cstdint>
#include <functional>

namespace rosflight_firmware
{
class ROSflight;

class CommManager :  public ParamListenerInterface
{
private:
  enum StreamId
  {
    STREAM_ID_HEARTBEAT,
    STREAM_ID_STATUS,

    STREAM_ID_ATTITUDE,

    STREAM_ID_IMU,
    STREAM_ID_DIFF_PRESSURE,
    STREAM_ID_BARO,
    STREAM_ID_SONAR,
    STREAM_ID_MAG,
    STREAM_ID_BATTERY_STATUS,

    STREAM_ID_SERVO_OUTPUT_RAW,
    STREAM_ID_GNSS,
    STREAM_ID_GNSS_FULL,
    STREAM_ID_RC_RAW,
    STREAM_ID_LOW_PRIORITY,
    STREAM_COUNT
  };

  enum OffboardControlMode
  {
    MODE_PASS_THROUGH,
    MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE,
    MODE_ROLL_PITCH_YAWRATE_THROTTLE,
    MODE_ROLL_PITCH_YAWRATE_ALTITUDE,
  };

  uint8_t sysid_;
  uint64_t offboard_control_time_;
  ROSflight & RF_;
  CommLinkInterface & comm_link_;
  uint8_t send_params_index_;
  bool initialized_ = false;
  bool connected_ = false;

  static constexpr int LOG_MSG_SIZE = 50;
  class LogMessageBuffer
  {
  public:
    static constexpr int LOG_BUF_SIZE = 25;
    LogMessageBuffer();

    struct LogMessage
    {
      char msg[LOG_MSG_SIZE];
      CommLinkInterface::LogSeverity severity;
    };
    void add_message(CommLinkInterface::LogSeverity severity, char msg[LOG_MSG_SIZE]);
    size_t size() const { return length_; }
    bool empty() const { return length_ == 0; }
    bool full() const { return length_ == LOG_BUF_SIZE; }
    const LogMessage & oldest() const { return buffer_[oldest_]; }
    void pop();

  private:
    LogMessage buffer_[LOG_BUF_SIZE];
    size_t oldest_ = 0;
    size_t newest_ = 0;
    size_t length_ = 0;
  };
  LogMessageBuffer log_buffer_;

  StateManager::BackupData backup_data_buffer_;
  bool have_backup_data_ = false;

  class Stream
  {
  public:
    Stream(uint32_t period_us, std::function<void(void)> send_function);

    void stream(uint64_t now_us);
    void set_rate(uint32_t rate_hz);

    uint32_t period_us_;
    uint64_t next_time_us_;
    std::function<void(void)> send_function_;
  };

  void update_system_id(uint16_t param_id);

  // The following are CommLinkInterface Listener Functions
  //
  //void param_request_list_callback(uint8_t target_system) override;
//  void param_request_read_callback(uint8_t target_system, const char * const param_name,
//                                   int16_t param_index) override;
//  void param_set_int_callback(uint8_t target_system, const char * const param_name,
//                              int32_t param_value) override;
//  void param_set_float_callback(uint8_t target_system, const char * const param_name,
//                                float param_value) override;
//  void command_callback(CommLinkInterface::Command command) override;
  //void timesync_callback(int64_t tc1, int64_t ts1) override;
  //void offboard_control_callback(const CommLinkInterface::OffboardControl & control) override;
  //void aux_command_callback(const CommLinkInterface::AuxCommand & command) override;
  //void external_attitude_callback(const turbomath::Quaternion & q) override;
  //void heartbeat_callback() override;

  void send_heartbeat(void);
  void send_status(void);
  void send_attitude(void);
  void send_imu(void);
  void send_output_raw(void);
  void send_rc_raw(void);
  void send_diff_pressure(void);
  void send_baro(void);
  void send_sonar(void);
  void send_mag(void);
  void send_battery_status(void);
  void send_gnss(void);
  void send_gnss_full(void);
  void send_low_priority(void);
  void send_rosflignt_cmd_ack(void);
  void send_named_value_int(const char * const name, int32_t value);
  void send_next_param(void);

  void receive_msg_offboard_control(CommLinkInterface::CommMessage *message);
  void receive_msg_param_request_list(CommLinkInterface::CommMessage *message);
  void receive_msg_param_request_read(CommLinkInterface::CommMessage *message);
  void receive_msg_param_set(CommLinkInterface::CommMessage *message);
  void receive_msg_rosflight_cmd(CommLinkInterface::CommMessage *message);
  void receive_msg_rosflight_aux_cmd(CommLinkInterface::CommMessage *message);
  void receive_msg_timesync(CommLinkInterface::CommMessage *message);
  void receive_msg_external_attitude(CommLinkInterface::CommMessage *message);
  void receive_msg_heartbeat(CommLinkInterface::CommMessage *message);

  // the time of week stamp for the last sent GNSS message, to prevent re-sending
  uint32_t last_sent_gnss_tow_ = 0;
  uint32_t last_sent_gnss_full_tow_ = 0;

public:
  CommManager(ROSflight & rf, CommLinkInterface & comm_link);

  void init();
  void param_change_callback(uint16_t param_id) override;
  void receive(void);
  void stream(got_flags got);
  void send_param_value(uint16_t param_id);
  void update_status();
  void log(CommLinkInterface::LogSeverity severity, const char * fmt, ...);
  void log_message(CommLinkInterface::LogSeverity severity, char * text);

  void send_named_value_float(const char * const name, float value);

  void send_backup_data(const StateManager::BackupData & backup_data);
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_COMM_MANAGER_H
