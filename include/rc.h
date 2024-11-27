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

#ifndef ROSFLIGHT_FIRMWARE_RC_H
#define ROSFLIGHT_FIRMWARE_RC_H

#include <cstdbool>
#include <cstdint>

#include "rosflight_structs.h"

namespace rosflight_firmware
{
class ROSflight;

class RC
{
public:
  enum Stick
  {
    STICK_X,
    STICK_Y,
    STICK_Z,
    STICK_F,
    STICKS_COUNT
  };

  enum Switch
  {
    SWITCH_ARM,
    SWITCH_ATT_OVERRIDE,
    SWITCH_THROTTLE_OVERRIDE,
    SWITCH_ATT_TYPE,
    SWITCHES_COUNT
  };

  RC(ROSflight & _rf);

  void init();
  float stick(Stick channel);
  bool switch_on(Switch channel);
  bool switch_mapped(Switch channel);
  bool run();
  bool new_command();
  void param_change_callback(uint16_t param_id);

  RcStruct * get_rc(void);
  float read_chan(uint8_t chan);
  bool receive(void);

  // _FAKE_ RC receive. Use set_chan for testing only!
  uint16_t fake_rx(uint16_t *chan, uint16_t len, bool lost, bool failsafe);

private:
  RcStruct rc_={};

  ROSflight & RF_;

  typedef struct
  {
    uint8_t channel;
    int8_t direction;
    bool mapped;
  } rc_switch_config_t;

  typedef struct
  {
    uint8_t channel;
    bool one_sided;
  } rc_stick_config_t;

  bool new_command_;

  uint32_t time_of_last_stick_deviation = 0;
  uint32_t time_sticks_have_been_in_arming_position_ms = 0;
  uint32_t prev_time_ms = 0;
  uint32_t last_rc_receive_time = 0;

  rc_stick_config_t sticks[STICKS_COUNT];
  rc_switch_config_t switches[SWITCHES_COUNT];

  volatile bool switch_values[SWITCHES_COUNT];
  volatile float stick_values[STICKS_COUNT];

  void init_rc();
  void init_switches();
  void init_sticks();
  bool check_rc_lost();
  void look_for_arm_disarm_signal();
};

} // namespace rosflight_firmware

#endif // ROSLFLIGHT_FIRMWARE_RC_H
