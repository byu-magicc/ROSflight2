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

#ifndef ROSFLIGHT_FIRMWARE_TEST_BOARD_H
#define ROSFLIGHT_FIRMWARE_TEST_BOARD_H

#include "board.h"
#include "sensors.h"

namespace rosflight_firmware
{
class testBoard : public Board
{
private:
  uint16_t rc_values[8] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500};
  bool new_rc_ = false;
  uint64_t time_us_ = 0;
  float acc_[3] = {0, 0, 0};
  float gyro_[3] = {0, 0, 0};
  bool new_imu_ = false;
  static constexpr size_t BACKUP_MEMORY_SIZE{1024};
  uint8_t backup_memory_[BACKUP_MEMORY_SIZE];

public:
  // setup

  void init_board(void) override;
  void board_reset(bool bootloader) override;

  // clock
  uint32_t clock_millis() override;
  uint64_t clock_micros() override;
  void clock_delay(uint32_t milliseconds) override;

  // serial
  void serial_init(uint32_t baud_rate, uint32_t dev) override;
  void serial_write(const uint8_t * src, size_t len, uint8_t qos) override;
  uint16_t serial_bytes_available() override;
  uint8_t serial_read() override;
  void serial_flush() override;

  // sensors
  void sensors_init() override;
  uint16_t sensors_errors_count() override;
  uint16_t sensors_init_message_count() override;
  uint16_t sensors_init_message(char * message, uint16_t size, uint16_t i) override;
  bool sensors_init_message_good(uint16_t i) override;

  bool imu_read(rosflight_firmware::ImuStruct * imu) override;

  bool mag_read(rosflight_firmware::MagStruct * mag) override;

  bool baro_read(rosflight_firmware::PressureStruct * baro) override;

  bool diff_pressure_read(rosflight_firmware::PressureStruct * diff_pressure) override;

  bool sonar_read(rosflight_firmware::RangeStruct * sonar) override;

  // Battery
  bool battery_read(rosflight_firmware::BatteryStruct * bat) override;
  void battery_voltage_set_multiplier(double multiplier) override;
  void battery_current_set_multiplier(double multiplier) override;

  // GNSS
  bool gnss_read(rosflight_firmware::GnssStruct * gnss) override;

  // RC
  void rc_init(rc_type_t rc_type) override;
  bool rc_read(rosflight_firmware::RcStruct * rc) override;

  // PWM
  void pwm_init(const float * rate, uint32_t channels) override;
  void pwm_disable() override;
  void pwm_write(float * value, uint32_t channels) override;

  // non-volatile memory
  void memory_init() override;
  bool memory_read(void * dest, size_t len) override;
  bool memory_write(const void * src, size_t len) override;

  // LEDs
  void led0_on() override;
  void led0_off() override;
  void led0_toggle() override;

  void led1_on() override;
  void led1_off() override;
  void led1_toggle() override;

  // Backup Data
  void backup_memory_init() override;
  bool backup_memory_read(void * dest, size_t len) override;
  void backup_memory_write(const void * src, size_t len) override;
  void backup_memory_clear(size_t len) override;
  void backup_memory_clear();

  void set_imu(float * acc, float * gyro, uint64_t time_us);
  void set_time(uint64_t time_us);
  void set_pwm_lost(bool lost);
};

} // namespace rosflight_firmware

#endif // ROSLFIGHT_FIRMWARE_TEST_BOARD_H
