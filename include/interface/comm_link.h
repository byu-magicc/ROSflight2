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

#ifndef ROSFLIGHT_FIRMWARE_COMM_LINK_H
#define ROSFLIGHT_FIRMWARE_COMM_LINK_H

#include "param.h"
#include "sensors.h"
#include "state_manager.h"
#include "mixer.h"

#include <turbomath/turbomath.h>

#include <cstdint>

namespace rosflight_firmware
{
class CommLinkInterface
{
#define cast_in_range(val,type) (static_cast<uint32_t>(val) < static_cast<uint32_t>(type::END)) ? static_cast<type>(val) : type::END

public:
  enum class LogSeverity
  {
    LOG_INFO=0, // c.f., MAVlink MAV_SEVERITY_INFO=6,
    LOG_WARNING, // c.f., MAVlink MAV_SEVERITY_WARNING=4,
    LOG_ERROR, // c.f., MAVlink MAV_SEVERITY_ERROR=3,
    LOG_CRITICAL, // c.f., MAVlink MAV_SEVERITY_CRITICAL=2,
    END // c.f., MAVlink
  };

  enum class CommMessageType
  {
    MESSAGE_OFFBOARD_CONTROL=0,
    MESSAGE_PARAM_REQUEST_LIST,
    MESSAGE_PARAM_REQUEST_READ,
    MESSAGE_PARAM_SET,
    MESSAGE_ROSFLIGHT_CMD,
    MESSAGE_ROSFLIGHT_AUX_CMD,
    MESSAGE_TIMESYNC,
    MESSAGE_EXTERNAL_ATTITUDE,
    MESSAGE_HEARTBEAT,
    END
  };

  enum class CommMessageCommand // c.f., MAVlink ROSFLIGHT_CMD
  {
    ROSFLIGHT_CMD_RC_CALIBRATION=0, /*  | */
    ROSFLIGHT_CMD_ACCEL_CALIBRATION=1, /*  | */
    ROSFLIGHT_CMD_GYRO_CALIBRATION=2, /*  | */
    ROSFLIGHT_CMD_BARO_CALIBRATION=3, /*  | */
    ROSFLIGHT_CMD_AIRSPEED_CALIBRATION=4, /*  | */
    ROSFLIGHT_CMD_READ_PARAMS=5, /*  | */
    ROSFLIGHT_CMD_WRITE_PARAMS=6, /*  | */
    ROSFLIGHT_CMD_SET_PARAM_DEFAULTS=7, /*  | */
    ROSFLIGHT_CMD_REBOOT=8, /*  | */
    ROSFLIGHT_CMD_REBOOT_TO_BOOTLOADER=9, /*  | */
    ROSFLIGHT_CMD_SEND_VERSION=10, /*  | */
    ROSFLIGHT_CMD_RESET_ORIGIN=11, /*  | */
    ROSFLIGHT_CMD_SEND_ALL_CONFIG_INFOS=12, /*  | */
    END=13, /*  | */
   };

  enum class RosflightAuxCmdType // c.f., MAVlink ROSFLIGHT_AUX_CMD_TYPE
  {
    DISABLED=0, /*  | */
    SERVO=1, /*  | */
    MOTOR=2, /*  | */
    END=3
  };

  enum class RosflightCmdResponse // c.f., MAVlink ROSFLIGHT_CMD_RESPONSE
  {
    ROSFLIGHT_CMD_FAILED=0, /*  | */
    ROSFLIGHT_CMD_SUCCESS=1, /*  | */
    END=2
  };

  enum class OffboardControlMode // c.f., MAVlink OFFBOARD_CONTROL_MODE
  {
    MODE_PASS_THROUGH=0, /* Pass commanded values directly to actuators | */
    MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE=1, /* Command roll rate, pitch rate, yaw rate, and throttle | */
    MODE_ROLL_PITCH_YAWRATE_THROTTLE=2, /* Command roll angle, pitch angle, yaw rate, and throttle | */
    MODE_ROLL_PITCH_YAWRATE_ALTITUDE=3, /* Command roll angle, pitch angle, yaw rate, and altitude above ground | */
    MODE_XVEL_YVEL_YAWRATE_ALTITUDE=4, /* Command body-fixed, x and y velocity, and yaw rate, and altitude above ground | */
    MODE_XPOS_YPOS_YAW_ALTITUDE=5, /* Command inertial x, y position (m) wrt origin, yaw angle wrt north, and altitude above ground | */
    END=6, /*  | */
  };

 enum class OffboardControlIgnore // c.f., MAVlink OFFBOARD_CONTROL_IGNORE
  {
    IGNORE_NONE =0x00, /* Convenience value for specifying no fields should be ignored | */
    IGNORE_VALUE1=0x01, /* Field value1 should be ignored | */
    IGNORE_VALUE2=0x02, /* Field value2 should be ignored | */
    IGNORE_VALUE3=0x04, /* Field value3 should be ignored | */
    IGNORE_VALUE4=0x08, /* Field value4 should be ignored | */
    IGNORE_VALUE5=0x10, /* Field value4 should be ignored | */
    IGNORE_VALUE6=0x20, /* Field value4 should be ignored | */
    END = 9
  };

// enum class RosflightErrorCode // c.f., ROSFLIGHT_ERROR_CODE
// {
//   ROSFLIGHT_ERROR_NONE=0, /*  | */
//   ROSFLIGHT_ERROR_INVALID_MIXER=1, /*  | */
//   ROSFLIGHT_ERROR_IMU_NOT_RESPONDING=2, /*  | */
//   ROSFLIGHT_ERROR_RC_LOST=4, /*  | */
//   ROSFLIGHT_ERROR_UNHEALTHY_ESTIMATOR=8, /*  | */
//   ROSFLIGHT_ERROR_TIME_GOING_BACKWARDS=16, /*  | */
//   ROSFLIGHT_ERROR_UNCALIBRATED_IMU=32, /*  | */
//   ROSFLIGHT_ERROR_BUFFER_OVERRUN=64, /*  | */
//   END=65, /*  | */
// };

  enum class RosFlightRangeType // c.f., ROSFLIGHT_RANGE_TYPE
  {
    ROSFLIGHT_RANGE_SONAR=0, /*  | */
    ROSFLIGHT_RANGE_LIDAR=1, /*  | */
    END=2, /*  | */
  };

//  enum class GnssFixType // c.f., MAVlink GNSS_FIX_TYPE
//  {
//    GNSS_FIX_NO_FIX=0, /*  | */
//    GNSS_FIX_FIX=1, /*  | */
//    GNSS_FIX_RTK_FLOAT=2, /*  | */
//    GNSS_FIX_RTK_FIXED=3, /*  | */
//    END=4, /*  | */
//  };

//  enum class UbloxFixType // c.f., UBLOX_FIX_TYPE
//  {
//    NO_FIX=0, /*  | */
//    DEAD_RECKONING_ONLY=1, /*  | */
//    FIX_2D=2, /*  | */
//    FIX_3D=3, /*  | */
//    GNSS_AND_DEAD_RECKONING=4, /*  | */
//    TIME_ONLY=5, /*  | */
//    END=6, /*  | */
//  };

  typedef struct
  {
    CommMessageType type;
    union
    {
      struct
      {
        OffboardControlMode mode;
    struct Channel
    {
      float value;
      bool valid;
    };
        Channel U[6]; // allow for 3 moments and 3 forces.
      } offboard_control_;

      struct
      {
        int16_t id; /*< Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)*/
        char name[Params::PARAMS_NAME_LENGTH]; /*< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
      } param_read_;

      struct
  {
        param_value_t value;
        char name[Params::PARAMS_NAME_LENGTH];
        param_type_t type;
      } param_set_;

      struct
    {
        float q[4]; // w,x,y,z
      } external_attitude_quaternion_;

      struct //
    {
        CommMessageCommand command;
        uint8_t success;
      } rosflight_cmd_;

      Mixer::aux_command_t new_aux_command_;

      struct
  {
        uint64_t local;
        uint64_t remote;
      } time_sync_;
  };
  } CommMessage;

  virtual void init(uint32_t baud_rate, uint32_t dev) = 0;

  // receive function

  virtual bool parse_char(uint8_t ch, CommMessage *message)=0;

  // send functions

  virtual void send_attitude_quaternion(uint8_t system_id, uint64_t timestamp_us,
                                        const turbomath::Quaternion & attitude,
                                        const turbomath::Vector & angular_velocity) = 0;
  virtual void send_baro(uint8_t system_id, float altitude, float pressure, float temperature) = 0;
  virtual void send_command_ack(uint8_t system_id, CommMessageCommand command, RosflightCmdResponse success) = 0;
  virtual void send_diff_pressure(uint8_t system_id, float velocity, float pressure,
                                  float temperature) = 0;
  virtual void send_heartbeat(uint8_t system_id, bool fixed_wing) = 0;
  virtual void send_imu(uint8_t system_id, uint64_t timestamp_us, const turbomath::Vector & accel,
                        const turbomath::Vector & gyro, float temperature) = 0;
  virtual void send_log_message(uint8_t system_id, LogSeverity severity, const char * text) = 0;
  virtual void send_mag(uint8_t system_id, const turbomath::Vector & mag) = 0;
  virtual void send_named_value_int(uint8_t system_id, uint32_t timestamp_ms,
                                    const char * const name, int32_t value) = 0;
  virtual void send_named_value_float(uint8_t system_id, uint32_t timestamp_ms,
                                      const char * const name, float value) = 0;
  virtual void send_output_raw(uint8_t system_id, uint32_t timestamp_ms,
                               const float raw_outputs[14]) = 0;
  virtual void send_param_value_int(uint8_t system_id, uint16_t index, const char * const name,
                                    int32_t value, uint16_t param_count) = 0;
  virtual void send_param_value_float(uint8_t system_id, uint16_t index, const char * const name,
                                      float value, uint16_t param_count) = 0;
  virtual void send_rc_raw(uint8_t system_id, uint32_t timestamp_ms,
                           const uint16_t channels[8]) = 0;
  virtual void send_sonar(uint8_t system_id,
                          /* TODO enum type*/ uint8_t type, float range, float max_range,
                          float min_range) = 0;
  virtual void send_status(uint8_t system_id, bool armed, bool failsafe, bool rc_override,
                           bool offboard, uint8_t error_code, uint8_t control_mode,
                           int16_t num_errors, int16_t loop_time_us) = 0;
  virtual void send_timesync(uint8_t system_id, int64_t tc1, int64_t ts1) = 0;
  virtual void send_version(uint8_t system_id, const char * const version) = 0;
  virtual void send_gnss(uint8_t system_id, const GNSSData & data) = 0;
  virtual void send_gnss_full(uint8_t system_id, const GNSSFull & data) = 0;
  virtual void send_error_data(uint8_t system_id, const StateManager::BackupData & error_data) = 0;
  virtual void send_battery_status(uint8_t system_id, float voltage, float current) = 0;
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_COMM_LINK_H
