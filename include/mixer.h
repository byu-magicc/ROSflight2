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

#ifndef ROSFLIGHT_FIRMWARE_MIXER_H
#define ROSFLIGHT_FIRMWARE_MIXER_H

#include "interface/param_listener.h"

#include <cstdbool>
#include <cstdint>

namespace rosflight_firmware
{
class ROSflight;

class Mixer : public ParamListenerInterface
{

public:
  static constexpr uint8_t NUM_TOTAL_OUTPUTS = 14;
  static constexpr uint8_t NUM_MIXER_OUTPUTS = 10;
  float * raw_outputs(void);

  enum
  {
    ESC_CALIBRATION = 0,
    QUADCOPTER_PLUS = 1,
    QUADCOPTER_X = 2,
    HEX_PLUS = 3,
    HEX_X = 4,
    OCTO_PLUS = 5,
    OCTO_X = 6,
    Y6 = 7,
    X8 = 8,
    TRICOPTER = 9,
    FIXEDWING = 10,
    PASSTHROUGH = 11,
    VTAIL = 12,
    QUADPLANE = 13,
    CUSTOM = 14,
    NUM_MIXERS,
    INVALID_MIXER = 255
  };

  typedef enum
  {
    NONE, // None
    S,    // Servo
    M,    // Motor
    G     // GPIO
  } output_type_t;

  typedef struct
  {
    output_type_t output_type[NUM_MIXER_OUTPUTS];
    float default_pwm_rate[NUM_MIXER_OUTPUTS];
    float F[NUM_MIXER_OUTPUTS];
    float x[NUM_MIXER_OUTPUTS];
    float y[NUM_MIXER_OUTPUTS];
    float z[NUM_MIXER_OUTPUTS];
  } mixer_t;

  typedef struct
  {
    output_type_t type;
    float value;
  } aux_channel_t;

  typedef struct
  {
    aux_channel_t channel[NUM_TOTAL_OUTPUTS];
  } aux_command_t;

private:
  ROSflight & RF_;

  float raw_outputs_[NUM_TOTAL_OUTPUTS];
  float outputs_[NUM_TOTAL_OUTPUTS];
  aux_command_t aux_command_;
  output_type_t combined_output_type_[NUM_TOTAL_OUTPUTS];

  // clang-format off

  const mixer_t esc_calibration_mixing = {
    {   M,    M,    M,    M,    M,    M, NONE, NONE, NONE, NONE},  // output type
    {  50,   50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},  // F Mix
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // X Mix

  const mixer_t quadcopter_plus_mixing = {
    {M, M, M, M, NONE, NONE, NONE, NONE, NONE, NONE}, // output_type
    { 490,   490,   490,   490,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {  1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},   // F Mix
    {  0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // X Mix
    {  1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Y Mix
    {  1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t quadcopter_x_mixing = {
    {    M,     M,     M,     M, NONE, NONE, NONE, NONE, NONE, NONE},  // output_type
    {  490,   490,   490,   490,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
	{1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},   // F Mix
	{-1.0f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
	{1.0f, -1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
	{1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t hex_plus_mixing = {
    {   M,          M,          M,     M,         M,         M, NONE, NONE, NONE, NONE},  // output_type
    { 490,        490,        490,   490,       490,       490,   50,   50,   50,   50},  // Rate (Hz)
    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},                       //  F  Mix
    {0.0f, -0.866025f, -0.866025f, 0.0f, 0.866025f, 0.866025f, 0.0f, 0.0f, 0.0f, 0.0f}, //  X  Mix
    {1.0f, 0.5f, -0.5f, -1.0f, -0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f},                    //  Y  Mix
    {1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f}};                    //  Z  Mix

  const mixer_t hex_x_mixing = {
    {        M,     M,          M,          M,    M,         M, NONE, NONE, NONE, NONE},  // output_type
    {      490,   490,        490,        490,  490,       490,   50,   50,   50,   50},  // Rate (Hz)
    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},                       //  F  Mix
    {-0.5f, -1.0f, -0.5f, 0.5f, 1.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f},                    //  X  Mix
    {0.866025f, 0.0f, -0.866025f, -0.866025f, 0.0f, 0.866025f, 0.0f, 0.0f, 0.0f, 0.0f}, //  Y  Mix
    {1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f} };                   //  Z  Mix

  const mixer_t octocopter_plus_mixing = {
    {M, M, M, M, M, M, M, M, NONE, NONE}, // output_type
    { 490,     490,   490,     490,   490,     490,  490,    490,   50,   50},  // Rate (Hz)
    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f},            //  F  Mix
    {0.0f, -0.707f, -1.0f, -0.707f, 0.0f, 0.707f, 1.0f, 0.707f, 0.0f, 0.0f}, //  X  Mix
    {1.0f, 0.707f, 0.0f, -0.707f, -1.0f, -0.707f, 0.0f, 0.707f, 0.0f, 0.0f}, //  Y  Mix
    {1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f}};        //  Z  Mix

  const mixer_t octocopter_x_mixing = {
    {M, M, M, M, M, M, M, M, NONE, NONE}, // output_type
    {    490,    490,     490,     490,    490,     490,    490,   490,   50,   50},  // Rate (Hz)
    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f},            // F Mix
    {-0.414f, -1.0f, -1.0f, -0.414f, 0.414f, 1.0f, 1.0f, 0.414, 0.0f, 0.0f}, // X Mix
    {1.0f, 0.414f, -0.414f, -1.0f, -1.0f, -0.414f, 0.414f, 1.0, 0.0f, 0.0f}, // Y Mix
    {1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f}};        // Z Mix

  const mixer_t Y6_mixing = {
	{M, M, M, M, M, M, NONE, NONE, NONE, NONE}, // output_type
    {   490,    490,     490,     490,    490,    490,   50,   50,   50,   50},  // Rate (Hz)
	{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},               // F Mix
	{-1.0f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},             // X Mix
	{0.667f, 0.667f, -1.333f, -1.333f, 0.667f, 0.667f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
	{1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f}};            // Z Mix

  const mixer_t X8_mixing = {
	{M, M, M, M, M, M, M, M, NONE, NONE}, // output_type
    {  490,   490,   490,   490,   490,   490,  490,  490,   50,   50},  // Rate (Hz)
	{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f},     // F Mix
	{-1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f}, // X Mix
	{1.0f, 1.0f, -1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f}, // Y Mix
	{1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f} };// Z Mix

  const mixer_t tricopter_mixing = {
	{      M,       M,      M,    NONE,        S,   NONE,   NONE,   NONE,   NONE,   NONE}, // output_type
    {    490,     490,    490,      50,       50,     50,     50,     50,     50,     50},  // Rate (Hz)
	{ 1.000f,  0.000f, 1.000f,  0.000f,   1.000f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f}, // F Mix
	{-1.000f,  0.000f, 0.000f,  0.000f,   1.000f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f}, // X Mix
	{ 0.667f,  0.000f, 0.667f,  0.000f,  -1.333f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f}, // Y Mix
	{ 0.000f,  1.000f, 0.000f,  0.000f,   0.000f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f} }; // Z Mix

  const mixer_t fixedwing_mixing = {
	{   S,    S,    S, NONE,    M, NONE, NONE, NONE, NONE, NONE},  // output type
    { 50,    50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F Mix
	{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // X Mix
	{0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Y Mix
	{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t fixedwing_vtail_mixing = {
    {   S, NONE, NONE,     S,    S,    M, NONE, NONE, NONE, NONE},  // Ailerons, LRuddervator, RRuddervator, Motor
    {  50,    50,   50,   50,   50,   50,   50,   50,   50,   50},  // Rate (Hz)
    {0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // F Mix
    {1.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // X Mix
    {0.0f, 0.0f, 0.0f, -0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // Y Mix
    {0.0f, 0.0f, 0.0f,  0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t quadplane_mixing = {
    {   S,     S,    S,    M,    M,    M,    M,    M, NONE, NONE},  // Ailerons, Rudder, Elevator, Tractor Motor, Quadrotors
    {  50,    50,   50,   50,  490,   490,   490,   490,   50,   50},  // Rate (Hz)
    {0.0f,  0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f},  // F Mix
    {1.0f,  0.0f, 0.0f, 0.0f, 0.0f,-1.0f, 0.0f, 1.0f, 0.0f, 0.0f},  // X Mix
    {0.0f,  1.0f, 0.0f, 0.0f, 1.0f, 0.0f,-1.0f, 0.0f, 0.0f, 0.0f},  // Y Mix
    {0.0f,  0.0f, 1.0f, 0.0f, 1.0f,-1.0f, 1.0f,-1.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t passthrough_mixing = {
	{NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE},
	{ 50,  50,  50,  50,  50,  50,  50,  50,  50,  50}, // Rate (Hz or kHz)
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t custom_mixing = {
	{NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE}, // output type
	{ 50,  50,  50,  50,  50,  50,  50,  50,  50,  50}, // Rate (Hz or kHz)
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}}; // Z Mix

  const mixer_t * mixer_to_use_;

  const mixer_t* array_of_mixers_[NUM_MIXERS] = {
		&esc_calibration_mixing,
		&quadcopter_plus_mixing,
		&quadcopter_x_mixing,
		&hex_plus_mixing,
		&hex_x_mixing,
		&octocopter_plus_mixing,
		&octocopter_x_mixing,
		&Y6_mixing,
		&X8_mixing,
		&tricopter_mixing,
		&fixedwing_mixing,
		&passthrough_mixing,
		&fixedwing_vtail_mixing,
		&quadplane_mixing,
		&custom_mixing
	};
  // clang-format on

public:
  Mixer(ROSflight & _rf);
  void init();
  void init_PWM();
  void init_mixing();
  void mix_output();
  void param_change_callback(uint16_t param_id) override;
  void set_new_aux_command(aux_command_t new_aux_command);
  inline const float * get_outputs() const { return raw_outputs_; }
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_MIXER_H
