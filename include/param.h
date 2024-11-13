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

#ifndef ROSFLIGHT_FIRMWARE_PARAM_H
#define ROSFLIGHT_FIRMWARE_PARAM_H

#ifndef GIT_VERSION_HASH
#define GIT_VERSION_HASH 0x00
#pragma message "GIT_VERSION_HASH Undefined, setting to 0x00!"
#endif
#ifndef GIT_VERSION_STRING
#define GIT_VERSION_STRING "empty"
#pragma message "GIT_VERSION_STRING Undefined, setting to \"empty\"!"
#endif

// Uncomment to view contents of GIT_VERSION_HASH and GIT_VERSION STRING
// #define STRINGIFY(s) XSTRINGIFY(s)
// #define XSTRINGIFY(s) #s
// #pragma message( "GIT_VERSION_HASH: " STRINGIFY(GIT_VERSION_HASH))
// #pragma message( "GIT_VERSION_STRING: " GIT_VERSION_STRING)

#include "interface/param_listener.h"

#include <cstddef>
#include <cstdint>

namespace rosflight_firmware
{
enum : uint16_t
{
  /******************************/
  /*** HARDWARE CONFIGURATION ***/
  /******************************/
  PARAM_BAUD_RATE = 0,
  PARAM_SERIAL_DEVICE,

  /*****************************/
  /*** MAVLINK CONFIGURATION ***/
  /*****************************/
  PARAM_SYSTEM_ID,

  /********************************/
  /*** CONTROLLER CONFIGURATION ***/
  /********************************/
  PARAM_MAX_COMMAND,

  PARAM_PID_ROLL_RATE_P,
  PARAM_PID_ROLL_RATE_I,
  PARAM_PID_ROLL_RATE_D,

  PARAM_PID_PITCH_RATE_P,
  PARAM_PID_PITCH_RATE_I,
  PARAM_PID_PITCH_RATE_D,

  PARAM_PID_YAW_RATE_P,
  PARAM_PID_YAW_RATE_I,
  PARAM_PID_YAW_RATE_D,

  PARAM_PID_ROLL_ANGLE_P,
  PARAM_PID_ROLL_ANGLE_I,
  PARAM_PID_ROLL_ANGLE_D,

  PARAM_PID_PITCH_ANGLE_P,
  PARAM_PID_PITCH_ANGLE_I,
  PARAM_PID_PITCH_ANGLE_D,

  PARAM_X_EQ_TORQUE,
  PARAM_Y_EQ_TORQUE,
  PARAM_Z_EQ_TORQUE,

  PARAM_PID_TAU,

  /*************************/
  /*** PWM CONFIGURATION ***/
  /*************************/
  PARAM_MOTOR_PWM_SEND_RATE,
  PARAM_MOTOR_IDLE_THROTTLE,
  PARAM_FAILSAFE_THROTTLE,
  PARAM_SPIN_MOTORS_WHEN_ARMED,

  /*******************************/
  /*** ESTIMATOR CONFIGURATION ***/
  /*******************************/
  PARAM_INIT_TIME,
  PARAM_FILTER_KP_ACC,
  PARAM_FILTER_KI,
  PARAM_FILTER_KP_EXT,
  PARAM_FILTER_ACCEL_MARGIN,

  PARAM_FILTER_USE_QUAD_INT,
  PARAM_FILTER_USE_MAT_EXP,
  PARAM_FILTER_USE_ACC,

  PARAM_CALIBRATE_GYRO_ON_ARM,

  PARAM_GYRO_XY_ALPHA,
  PARAM_GYRO_Z_ALPHA,
  PARAM_ACC_ALPHA,

  PARAM_GYRO_X_BIAS,
  PARAM_GYRO_Y_BIAS,
  PARAM_GYRO_Z_BIAS,
  PARAM_ACC_X_BIAS,
  PARAM_ACC_Y_BIAS,
  PARAM_ACC_Z_BIAS,
  PARAM_ACC_X_TEMP_COMP,
  PARAM_ACC_Y_TEMP_COMP,
  PARAM_ACC_Z_TEMP_COMP,

  PARAM_MAG_A11_COMP,
  PARAM_MAG_A12_COMP,
  PARAM_MAG_A13_COMP,
  PARAM_MAG_A21_COMP,
  PARAM_MAG_A22_COMP,
  PARAM_MAG_A23_COMP,
  PARAM_MAG_A31_COMP,
  PARAM_MAG_A32_COMP,
  PARAM_MAG_A33_COMP,
  PARAM_MAG_X_BIAS,
  PARAM_MAG_Y_BIAS,
  PARAM_MAG_Z_BIAS,

  PARAM_BARO_BIAS,
  PARAM_GROUND_LEVEL,

  PARAM_DIFF_PRESS_BIAS,

  /************************/
  /*** RC CONFIGURATION ***/
  /************************/
  PARAM_RC_TYPE,
  PARAM_RC_X_CHANNEL,
  PARAM_RC_Y_CHANNEL,
  PARAM_RC_Z_CHANNEL,
  PARAM_RC_F_CHANNEL,
  PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL,
  PARAM_RC_THROTTLE_OVERRIDE_CHANNEL,
  PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,
  PARAM_RC_ARM_CHANNEL,
  PARAM_RC_NUM_CHANNELS,

  PARAM_RC_SWITCH_5_DIRECTION,
  PARAM_RC_SWITCH_6_DIRECTION,
  PARAM_RC_SWITCH_7_DIRECTION,
  PARAM_RC_SWITCH_8_DIRECTION,

  PARAM_RC_OVERRIDE_DEVIATION,
  PARAM_OVERRIDE_LAG_TIME,
  PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,

  PARAM_RC_ATTITUDE_MODE,
  PARAM_RC_MAX_ROLL,
  PARAM_RC_MAX_PITCH,
  PARAM_RC_MAX_ROLLRATE,
  PARAM_RC_MAX_PITCHRATE,
  PARAM_RC_MAX_YAWRATE,

  /***************************/
  /*** FRAME CONFIGURATION ***/
  /***************************/
  PARAM_MIXER,

  PARAM_FIXED_WING,
  PARAM_ELEVATOR_REVERSE,
  PARAM_AILERON_REVERSE,
  PARAM_RUDDER_REVERSE,

  PARAM_FC_ROLL,
  PARAM_FC_PITCH,
  PARAM_FC_YAW,

  /********************/
  /*** ARMING SETUP ***/
  /********************/
  PARAM_ARM_THRESHOLD,

  /************************/
  /*** OFFBOARD CONTROL ***/
  /************************/
  PARAM_OFFBOARD_TIMEOUT,

  /***********************/
  /*** BATTERY MONITOR ***/
  /***********************/
  PARAM_BATTERY_VOLTAGE_MULTIPLIER,
  PARAM_BATTERY_CURRENT_MULTIPLIER,
  PARAM_BATTERY_VOLTAGE_ALPHA,
  PARAM_BATTERY_CURRENT_ALPHA,

  // keep track of size of params array
  PARAMS_COUNT
};

typedef enum
{
  PARAM_TYPE_INT32,
  PARAM_TYPE_FLOAT,
  PARAM_TYPE_INVALID
} param_type_t;

typedef union
{
  float fvalue;
  int32_t ivalue;
} param_value_t;

class ROSflight;
class Params
{
public:
  static constexpr uint8_t PARAMS_NAME_LENGTH = 16;

private:
  typedef struct
  {
    uint32_t version;
    uint16_t size;
    uint8_t magic_be; // magic number, should be 0xBE

    param_value_t values[PARAMS_COUNT];
    char names[PARAMS_COUNT][PARAMS_NAME_LENGTH];
    param_type_t types[PARAMS_COUNT];

    uint8_t magic_ef; // magic number, should be 0xEF
    uint8_t chk;      // XOR checksum
  } params_t;

  params_t params;
  ROSflight & RF_;

  void init_param_int(uint16_t id, const char name[PARAMS_NAME_LENGTH], int32_t value);
  void init_param_float(uint16_t id, const char name[PARAMS_NAME_LENGTH], float value);
  uint8_t compute_checksum(void);

  ParamListenerInterface * const * listeners_;
  size_t num_listeners_;

public:
  Params(ROSflight & _rf);

  // function declarations

  /**
   * @brief Initialize parameter values
   */
  void init();

  /**
   * @brief Set all parameters to default values
   */
  void set_defaults(void);

  /**
   * @brief Specify listeners for parameter changes
   * @param listeners An array of pointers to objects that implement the ParamListenerInterface
   * interface
   * @param num_listeners The length of the array passed as the listeners parameter
   */
  void set_listeners(ParamListenerInterface * const listeners[], size_t num_listeners);

  /**
   * @brief Read parameter values from non-volatile memory
   * @return True if successful, false otherwise
   */
  bool read(void);

  /**
   * @brief Write current parameter values to non-volatile memory
   * @return True if successful, false otherwise
   */
  bool write(void);

  /**
   * @brief Callback for executing actions that need to be taken when a parameter value changes
   * @param id The ID of the parameter that was changed
   */
  void change_callback(uint16_t id);

  /**
   * @brief Gets the id of a parameter from its name
   * @param name The name of the parameter
   * @return The ID of the parameter if the name is valid, PARAMS_COUNT otherwise (invalid ID)
   */
  uint16_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);

  /**
   * @brief Get the value of an integer parameter by id
   * @param id The ID of the parameter
   * @return The value of the parameter
   */
  inline int get_param_int(uint16_t id) const { return params.values[id].ivalue; }

  /**
   * @brief Get the value of a floating point parameter by id
   * @param id The ID of the parameter
   * @return The value of the parameter
   */
  inline float get_param_float(uint16_t id) const { return params.values[id].fvalue; }

  /**
   * @brief Get the name of a parameter
   * @param id The ID of the parameter
   * @return The name of the parameter
   */
  inline const char * get_param_name(uint16_t id) const { return params.names[id]; }

  /**
   * @brief Get the type of a parameter
   * @param id The ID of the parameter
   * @return The type of the parameter
   * This returns one of three possible types
   * PARAM_TYPE_INT32, PARAM_TYPE_FLOAT, or PARAM_TYPE_INVALID
   * See line 165
   */
  inline param_type_t get_param_type(uint16_t id) const { return params.types[id]; }

  /**
   * @brief Sets the value of a parameter by ID and calls the parameter change callback
   * @param id The ID of the parameter
   * @param value The new value
   * @return True if a parameter value was changed, false otherwise
   */
  bool set_param_int(uint16_t id, int32_t value);

  /**
   * @brief Sets the value of a floating point parameter by ID and calls the parameter callback
   * @param id The ID of the parameter
   * @param value The new value
   * @return  True if a parameter was changed, false otherwise
   */
  bool set_param_float(uint16_t id, float value);

  /**
   * @brief Sets the value of a parameter by name and calls the parameter change callback
   * @param name The name of the parameter
   * @param value The new value
   * @return True if a parameter value was changed, false otherwise
   */
  bool set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value);

  /**
   * @brief Sets the value of a floating point parameter by name and calls the parameter change
   * callback
   * @param name The name of the parameter
   * @param value The new value
   * @return True if a parameter value was changed, false otherwise
   */
  bool set_param_by_name_float(const char name[PARAMS_NAME_LENGTH], float value);
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_PARAM_H
