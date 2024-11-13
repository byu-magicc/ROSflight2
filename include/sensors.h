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

#ifndef ROSFLIGHT_FIRMWARE_SENSORS_H
#define ROSFLIGHT_FIRMWARE_SENSORS_H

#include "interface/param_listener.h"

#include <turbomath/turbomath.h>

#include <cstdbool>
#include <cstdint>
#include <cstring>

#include <estimator.h>

namespace rosflight_firmware
{

typedef struct //__attribute__((__packed__))
{
  uint64_t timestamp; // us, time of data read complete
  float voltage;
  float current;
  float temperature; // STM32 temperature, not batter temperature
} BatteryStruct;

typedef struct //__attribute__((__packed__))
{
  uint64_t timestamp; // us, time of data read complete
  float accel[3];     // rad/s
  float gyro[3];      // rad/s
  float temperature;  // K
} ImuStruct;

// User version in estimator.h
typedef Estimator::AttitudeStruct AttitudeStruct;

typedef struct //__attribute__((__packed__))
{
  uint64_t timestamp; // us, time of data read complete
  float pressure;     // Pa
  float temperature;  // K
  union
  {
    float altitude;
    float speed;
  };
} PressureStruct;

enum class SensorRangeType // c.f., ROSFLIGHT_RANGE_TYPE
{
  ROSFLIGHT_RANGE_SONAR = 0, /*  | */
  ROSFLIGHT_RANGE_LIDAR = 1, /*  | */
  END = 2,                   /*  | */
};

typedef struct //__attribute__((__packed__))
{
  uint64_t timestamp;   // us, time of data read complete
  float range;          // m
  float min_range;      // m
  float max_range;      // m
  SensorRangeType type; // ROSFLIGHT_RANGE_SONAR, ROSFLIGHT_RANGE_SONAR
} RangeStruct;

typedef struct //__attribute__((packed))
{
  uint64_t timestamp; // us, time of data read complete
  float flux[3];      // T, magnetic flux density
  float temperature;  // K
} MagStruct;

#define RC_STRUCT_CHANNELS                                                                         \
  24           // 16 analog + 8 digital MUST BE > 14 (Mavlink message size is hardware to 14)
typedef struct //__attribute__((packed))
{
  uint64_t timestamp; // us, time of data read complete
  uint8_t nChan;
  float chan[RC_STRUCT_CHANNELS];
  bool frameLost;
  bool failsafeActivated;
} RcStruct;

enum class GNSSFixType // quality from GGA
{
  GNSS_FIX_TYPE_NO_FIX = 0,
  GNSS_FIX_TYPE_DEAD_RECKONING_ONLY = 1,
  GNSS_FIX_TYPE_2D_FIX = 2,
  GNSS_FIX_TYPE_3D_FIX = 3,
  GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING = 4,
  GNSS_FIX_TYPE_TIME_FIX_ONLY = 5,
  GNSS_FIX_RTK_FLOAT = 6,
  GNSS_FIX_RTK_FIXED = 7,
  END = 8
};

typedef struct //__attribute__((__packed__))
{
  uint64_t timestamp; // us, time of data read complete
  uint64_t pps;       // most recent pps timestamp
  uint64_t time;      // Unix time, in seconds (redundant)
  // GPS Time
  uint32_t time_of_week; //     / PVT
  uint16_t year;         // RMC / PVT
  uint8_t month;         // RMC / PVT
  uint8_t day;           // RMC / PVT
  uint8_t hour;          // GGA RMC UTC Time / PVT
  uint8_t min;           // GGA RMC UTC Time / PVT
  uint8_t sec;           // GGA RMC UTC Time / PVT
  uint32_t nano;         // GGA RMC UTC Time (ms) / PVT nano
  uint32_t t_acc;
  int32_t lon;              // GGA RMC / PVT
  int32_t lat;              // GGA RMC / PVT
  int32_t height_ellipsoid; //GGA RMC (computed) / PVT
  int32_t height_msl;       // GGA / PVT
  uint32_t h_acc;           // GST (lat and lon) / PVT hAcc
  uint32_t v_acc;           // GST / PVT vAcc
  int32_t ground_speed;     // RMC / PVT gSpeed
  int32_t course;           // RMC / PVT headMot
  int32_t course_accy;
  int32_t vel_n;       // no / PVT (RMC compute from ground velocity)
  int32_t vel_e;       // no / PVT (RMC compute from ground velocity)
  int32_t vel_d;       // no / PVT
  uint32_t speed_accy; // no /PVT (sACC) speed accuracy
  uint32_t mag_var;    // RMC / PVT
  // Fix
  uint8_t fix_type; // RMC (posmode), compute from GGA(quality) /PVT flags
  uint8_t valid;    // RMC (status), compute from GGA (0 or 6)
  uint8_t num_sat;  // GGA
  uint16_t dop;     // GGA RMC / PVT (pdop)
  struct
  {
    int32_t x;      // cm // not available on NMEA
    int32_t y;      // cm
    int32_t z;      // cm
    uint32_t p_acc; // cm
    int32_t vx;     // cm/s
    int32_t vy;     // cm/s
    int32_t vz;     // cm/s
    uint32_t s_acc; // cm/s
  } ecef;
} GnssStruct;

typedef struct
{
  bool imu;
  bool gnss;
  bool gnss_full;
  bool baro;
  bool mag;
  bool diff_pressure;
  bool sonar;
  bool battery;
  bool rc;
} got_flags;

class ROSflight;

class Sensors : public ParamListenerInterface
{
public:
  PressureStruct * get_diff_pressure(void) { return &diff_pressure_; }
  PressureStruct * get_baro(void) { return &baro_; }
  RangeStruct * get_sonar(void) { return &sonar_; }
  ImuStruct * get_imu(void) { return &imu_; }
  BatteryStruct * get_battery(void) { return &battery_; }
  RcStruct * get_output_raw(void) { return &output_raw_; }
  RcStruct * get_rc_(void) { return &rc_; }
  MagStruct * get_mag(void) { return &mag_; }
  GnssStruct * get_gnss(void) { return &gnss_; }

  float read_rc_chan(uint8_t chan) { return rc_.chan[chan]; }

  Sensors(ROSflight & rosflight);

  void init();
  got_flags run();
  void param_change_callback(uint16_t param_id) override;

  // Calibration Functions
  bool start_imu_calibration(void);
  bool start_gyro_calibration(void);
  bool start_baro_calibration(void);
  bool start_diff_pressure_calibration(void);
  bool gyro_calibration_complete(void);

private:
  // Data
  PressureStruct diff_pressure_ = {};
  PressureStruct baro_ = {};
  RangeStruct sonar_ = {};
  ImuStruct imu_ = {};
  BatteryStruct battery_ = {};
  RcStruct output_raw_ = {};
  RcStruct rc_ = {};
  MagStruct mag_ = {};
  GnssStruct gnss_ = {};

  void rotate_imu_in_place(ImuStruct * imu, turbomath::Quaternion q);
  turbomath::Quaternion fcu_orientation_ = {1, 0, 0, 0};

  static const int SENSOR_CAL_DELAY_CYCLES;
  static const int SENSOR_CAL_CYCLES;
  static const float BARO_MAX_CALIBRATION_VARIANCE;
  static const float DIFF_PRESSURE_MAX_CALIBRATION_VARIANCE;

  enum : uint8_t
  {
    BAROMETER,
    GNSS,
    DIFF_PRESSURE,
    SONAR,
    MAGNETOMETER,
    BATTERY_MONITOR,
    NUM_LOW_PRIORITY_SENSORS
  };

  ROSflight & rf_;

  float accel_[3] = {0, 0, 0};
  float gyro_[3] = {0, 0, 0};

  bool calibrating_acc_flag_ = false;
  bool calibrating_gyro_flag_ = false;
  void init_imu();
  void calibrate_accel(void);
  void calibrate_gyro(void);
  void calibrate_baro(void);
  void calibrate_diff_pressure(void);
  void correct_imu(void);
  void correct_mag(void);
  void correct_baro(void);
  void correct_diff_pressure(void);
  void update_battery_monitor_multipliers(void);

  // IMU calibration
  uint16_t gyro_calibration_count_ = 0;
  turbomath::Vector gyro_sum_ = {0, 0, 0};
  uint16_t accel_calibration_count_ = 0;
  turbomath::Vector acc_sum_ = {0, 0, 0};
  const turbomath::Vector gravity_ = {0.0f, 0.0f, 9.80665f};
  float acc_temp_sum_ = 0.0f;
  turbomath::Vector max_ = {-1000.0f, -1000.0f, -1000.0f};
  turbomath::Vector min_ = {1000.0f, 1000.0f, 1000.0f};

  // Baro Calibration
  bool baro_calibrated_ = false;
  uint16_t baro_calibration_count_ = 0;
  uint32_t last_baro_cal_iter_ms_ = 0;
  float baro_calibration_mean_ = 0.0f;
  float baro_calibration_var_ = 0.0f;

  // Diff Pressure Calibration
  bool diff_pressure_calibrated_ = false;
  uint16_t diff_pressure_calibration_count_ = 0;
  uint32_t last_diff_pressure_cal_iter_ms_ = 0;
  float diff_pressure_calibration_mean_ = 0.0f;
  float diff_pressure_calibration_var_ = 0.0f;

  // Battery Monitor
  float battery_voltage_alpha_{0.995};
  float battery_current_alpha_{0.995};
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_SENSORS_H
