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

#ifndef ROSFLIGHT_FIRMWARE_ROSFLIGHT_H
#define ROSFLIGHT_FIRMWARE_ROSFLIGHT_H

#include "interfaces/comm_link.h"
#include "param_listener.h"

#include "board.h"
#include "comm_manager.h"
#include "command_manager.h"
#include "controller.h"
#include "estimator.h"
#include "mixer.h"
#include "param.h"
#include "rc.h"
#include "sensors.h"
#include "state_manager.h"

#include <cstdint>

namespace rosflight_firmware
{
class ROSflight
{
public:
  ROSflight(Board & board, CommLinkInterface & comm_link);

  Board & board_;
  CommManager comm_manager_;

  Params params_;

  CommandManager command_manager_;
  Controller controller_;
  Estimator estimator_;
  Mixer mixer_;
  RC rc_;
  Sensors sensors_;
  StateManager state_manager_;

  uint32_t loop_time_us;

  /**
   * @brief Main initialization routine for the ROSflight autopilot flight stack
   */
  void init();

  /**
   * @brief Main loop for the ROSflight autopilot flight stack
   */
  void run();

  uint32_t get_loop_time_us();

private:
  static constexpr size_t num_param_listeners_ = 7;
  ParamListenerInterface * const param_listeners_[num_param_listeners_] = {
    &comm_manager_, &command_manager_, &controller_, &estimator_, &mixer_, &rc_, &sensors_};
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_ROSFLIGHT_H
